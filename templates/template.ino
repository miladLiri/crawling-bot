#include <ESP32Servo.h> // ESP32Servo by Kevin Harrington, John K. Bennet
#include <Wire.h>
#include <LiquidCrystal_I2C.h> // Adafruit LiquidCrystal by Adafruit
#include <NewPing.h>           // NewPing by Tim Eckel
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
// All other libraries are included in ESP32 Dev Module by Espressif Systems

// EEPROM settings
#define EEPROM_SIZE 1    // 1 byte to store robot number (1-8)
#define ROBOT_NUM_ADDR 0 // EEPROM address for robot number

// Base names for AP and OTA
const char *base_ssid = "ESP32-AP-";
const char *base_ota_hostname = "ESP32-OTA-";
const char *ap_password = "12345678"; // Common password for all APs

// Pin definitions
const int triggerPin = 5;
const int echoPin = 17;
const int control_servo_down = 32;
const int control_servo_up = 33;
#define MAX_DISTANCE 200 // Maximum distance to check in cm

// Q-Learning Parameters
#define STATES 16        // 4 positions x 4 positions = 16 states
#define ACTIONS 4        // 4 possible actions
#define EPISODES 10      // Number of training episodes
#define LEARNING_RATE 0.8
#define GAMMA 0.8       // Discount factor

// Servo angle configurations
const int SERVO1_POSITIONS[] = {0, 20, 40, 60};    // Base servo positions
const int SERVO2_POSITIONS[] = {40, 85, 130, 175}; // Arm servo positions

// Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servodown;
Servo servoup;
NewPing sonar(triggerPin, echoPin, MAX_DISTANCE); // NewPing instance

// Global variables for dynamic AP and OTA names
char ssid[32];
char ota_hostname[32];
uint8_t robot_number = 0;

// FreeRTOS task handle for OTA
TaskHandle_t otaTaskHandle = NULL;

// Q-Learning matrices
float Q[STATES][ACTIONS];  // Q-table
float R[STATES][ACTIONS];  // Reward table

// Current robot state
int currentState = 0;
int servo1Angle = 0;
int servo2Angle = 40;
float previousDistance = 0;

// Training statistics
int successfulMoves = 0;
float totalReward = 0;

void saveRobotNumber(uint8_t number)
{
    EEPROM.write(ROBOT_NUM_ADDR, number);
    EEPROM.commit();
    Serial.print("Saved robot number: ");
    Serial.println(number);
}

uint8_t readRobotNumber()
{
    uint8_t number = EEPROM.read(ROBOT_NUM_ADDR);
    if (number < 1 || number > 8)
    {
        Serial.println("Invalid or uninitialized robot number. Please set number (1-8).");
        lcd.clear();
        lcd.print("Set Robot Num: 1-8");
        while (!Serial.available())
        {
            delay(100); // Wait for Serial input
        }
        number = Serial.parseInt();
        if (number >= 1 && number <= 8)
        {
            saveRobotNumber(number);
            lcd.clear();
            lcd.print("Robot Num Set: ");
            lcd.print(number);
            delay(2000);
        }
        else
        {
            Serial.println("Invalid input. Defaulting to robot number 1.");
            number = 1;
            saveRobotNumber(number);
        }
    }
    return number;
}

void setup_ap()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, ap_password);
    Serial.println("AP Started");
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
}

void setup_ota()
{
    ArduinoOTA.setHostname(ota_hostname);
    // ArduinoOTA.setPassword("admin"); // Optional: Set OTA password
    ArduinoOTA.onStart([]()
                       {
        Serial.println("OTA Start");
        lcd.clear();
        lcd.print("OTA Update Start"); });
    ArduinoOTA.onEnd([]()
                     {
        Serial.println("\nOTA End");
        lcd.clear();
        lcd.print("OTA Update Done"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        lcd.clear();
        lcd.print("OTA Progress: ");
        lcd.setCursor(0, 1);
        lcd.print((progress / (total / 100)));
        lcd.print("%"); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
        Serial.printf("Error[%u]: ", error);
        lcd.clear();
        lcd.print("OTA Error");
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
    Serial.print("OTA Hostname: ");
    Serial.println(ota_hostname);
}

// OTA task to run asynchronously
void otaTask(void *parameter)
{
    for (;;)
    {
        ArduinoOTA.handle();                 // Handle OTA updates
        vTaskDelay(10 / portTICK_PERIOD_MS); // Yield for 10ms
    }
}

float getDistance()
{
    float distance = sonar.ping_cm(); // Get distance in cm
    if (distance == 0)
    {
        Serial.println("Warning: No echo received from SRF module.");
        return -1;
    }
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance;
}

void healthCheck()
{
    // 1. Print Health Check on LCD
    lcd.clear();
    lcd.print("Health Check Start");
    delay(1000);

    // 2. Get distance and print it
    float distance = getDistance();
    lcd.clear();
    if (distance < 0)
    {
        lcd.print("SRF Error!");
    }
    else
    {
        lcd.print("Dist: ");
        lcd.print(distance);
        lcd.print(" cm");
    }
    delay(500);

    servodown.write(90);
    servoup.write(90);
    delay(1000);
    // 3. Move servos back to initial position
    servodown.write(0);
    servoup.write(180);
    lcd.clear();
    lcd.print("Servos Reset");
    delay(1000);

    // 4. Print completion message
    lcd.clear();
    lcd.print("Health Check Done");
    delay(1500);
    lcd.clear();
}

void moveServoSmooth(Servo &servo, int from, int to, int stepDelay = 10)
{
    if (from < to)
    {
        for (int a = from; a <= to; a += 2)
        {
            servo.write(a);
            delay(stepDelay);
        }
    }
    else
    {
        for (int a = from; a >= to; a -= 2)
        {
            servo.write(a);
            delay(stepDelay);
        }
    }
}

// Q-Learning helper functions
void initializeQLearning() {
    // Initialize Q-table with zeros
    for (int i = 0; i < STATES; i++) {
        for (int j = 0; j < ACTIONS; j++) {
            Q[i][j] = 0;
            R[i][j] = 0;
        }
    }
    
    // Set invalid actions to -1 (boundary constraints)
    for (int state = 0; state < STATES; state++) {
        int s1_idx = state / 4;
        int s2_idx = state % 4;
        
        // Action 0: Move servo1 up (decrease position)
        if (s1_idx == 0) R[state][0] = -1;
        
        // Action 1: Move servo1 down (increase position)
        if (s1_idx == 3) R[state][1] = -1;
        
        // Action 2: Move servo2 up (decrease position)
        if (s2_idx == 0) R[state][2] = -1;
        
        // Action 3: Move servo2 down (increase position)
        if (s2_idx == 3) R[state][3] = -1;
    }
    
    // Set goal state reward
    R[15][2] = 100;  // Goal state (lowest position)
}

void moveToState(int state) {
    int s1_idx = state / 4;  // Servo 1 position index
    int s2_idx = state % 4;  // Servo 2 position index
    
    int targetAngle1 = SERVO1_POSITIONS[s1_idx];
    int targetAngle2 = SERVO2_POSITIONS[s2_idx];
    
    moveServoSmooth(servodown, servo1Angle, targetAngle1, 5);
    moveServoSmooth(servoup, servo2Angle, targetAngle2, 5);
    
    servo1Angle = targetAngle1;
    servo2Angle = targetAngle2;
    currentState = state;
}

int selectAction(int state, float epsilon) {
    // Epsilon-greedy action selection
    if (random(100) < epsilon * 100) {
        // Explore: select random valid action
        int validActions[ACTIONS];
        int validCount = 0;
        
        for (int a = 0; a < ACTIONS; a++) {
            if (R[state][a] >= 0) {
                validActions[validCount++] = a;
            }
        }
        
        if (validCount > 0) {
            return validActions[random(validCount)];
        }
    }
    
    // Exploit: select best action
    int bestAction = -1;
    float maxQ = -1000;
    
    for (int a = 0; a < ACTIONS; a++) {
        if (R[state][a] >= 0 && Q[state][a] > maxQ) {
            maxQ = Q[state][a];
            bestAction = a;
        }
    }
    
    return bestAction;
}

int getNextState(int state, int action) {
    int s1_idx = state / 4;
    int s2_idx = state % 4;
    
    switch (action) {
        case 0: // Servo1 up
            if (s1_idx > 0) s1_idx--;
            break;
        case 1: // Servo1 down
            if (s1_idx < 3) s1_idx++;
            break;
        case 2: // Servo2 up
            if (s2_idx > 0) s2_idx--;
            break;
        case 3: // Servo2 down
            if (s2_idx < 3) s2_idx++;
            break;
    }
    
    return s1_idx * 4 + s2_idx;
}

float getMaxQ(int state) {
    float maxQ = -1000;
    for (int a = 0; a < ACTIONS; a++) {
        if (R[state][a] >= 0 && Q[state][a] > maxQ) {
            maxQ = Q[state][a];
        }
    }
    return maxQ;
}

void doTraining()
{
    // TODO: Add your training logic here
    lcd.clear();
    lcd.print("Training Start");
    lcd.setCursor(0, 1);
    lcd.print("Episodes: ");
    lcd.print(EPISODES);
    
    Serial.println("=== Starting Q-Learning Training ===");
    
    for (int episode = 0; episode < EPISODES; episode++) {
        Serial.print("Episode ");
        Serial.print(episode + 1);
        Serial.println(" starting...");
        
        // Reset to random starting state
        currentState = random(STATES);
        moveToState(currentState);
        delay(500);
        
        float episodeReward = 0;
        int steps = 0;
        float epsilon = 1.0 - (float)episode / EPISODES;  // Decrease exploration over time
        
        // Run episode until goal or max steps
        while (currentState != 15 && steps < 50) {
            float distBefore = getDistance();
            
            // Select and execute action
            int action = selectAction(currentState, epsilon);
            if (action == -1) break;  // No valid actions
            
            int nextState = getNextState(currentState, action);
            moveToState(nextState);
            delay(200);
            
            float distAfter = getDistance();
            float distMoved = distBefore - distAfter;  // Positive if moved forward
            
            // Calculate reward based on movement
            float reward = 0;
            if (distMoved > 1.5) {  // Moved forward significantly
                reward = map(distMoved, 1.5, 5, 10, 50);
                successfulMoves++;
                Serial.print("Good move! Distance: ");
                Serial.println(distMoved);
            } else if (distMoved < -1.5) {  // Moved backward
                reward = -10;
            }
            
            // Update reward table with movement reward
            if (R[currentState][action] >= 0) {
                R[currentState][action] = max(R[currentState][action], reward);
            }
            
            // Q-Learning update
            float maxQNext = getMaxQ(nextState);
            Q[currentState][action] = (1 - LEARNING_RATE) * Q[currentState][action] + 
                                      LEARNING_RATE * (reward + GAMMA * maxQNext);
            
            episodeReward += reward;
            currentState = nextState;
            steps++;
            
            // Update LCD
            lcd.setCursor(0, 1);
            lcd.print("Ep:");
            lcd.print(episode + 1);
            lcd.print(" St:");
            lcd.print(steps);
            lcd.print(" R:");
            lcd.print((int)episodeReward);
        }
        
        totalReward += episodeReward;
        Serial.print("Episode ");
        Serial.print(episode + 1);
        Serial.print(" completed. Reward: ");
        Serial.println(episodeReward);
        
        delay(1000);
    }
    
    Serial.println("=== Training Complete ===");
    Serial.print("Total successful moves: ");
    Serial.println(successfulMoves);
    Serial.print("Average reward per episode: ");
    Serial.println(totalReward / EPISODES);
    
    lcd.clear();
    lcd.print("Training Done!");
    lcd.setCursor(0, 1);
    lcd.print("Moves: ");
    lcd.print(successfulMoves);
    delay(2000);
}

void doLearnedBehavior()
{
    // TODO: Add your learned behavior logic here
    lcd.clear();
    lcd.print("Learned Behavior");
    Serial.println("\n=== Executing Learned Behavior ===");
    
    // Reset to starting position
    currentState = 0;
    moveToState(currentState);
    delay(1000);
    
    int steps = 0;
    float totalDistance = 0;
    float startDistance = getDistance();
    
    // Execute learned policy
    while (currentState != 15 && steps < 20) {
        // Select best action from Q-table
        int bestAction = -1;
        float maxQ = -1000;
        
        for (int a = 0; a < ACTIONS; a++) {
            if (R[currentState][a] >= 0 && Q[currentState][a] > maxQ) {
                maxQ = Q[currentState][a];
                bestAction = a;
            }
        }
        
        if (bestAction == -1) {
            Serial.println("No valid action found!");
            break;
        }
        
        Serial.print("State ");
        Serial.print(currentState);
        Serial.print(" -> Action ");
        Serial.print(bestAction);
        
        int nextState = getNextState(currentState, bestAction);
        moveToState(nextState);
        
        float currentDistance = getDistance();
        Serial.print(" -> State ");
        Serial.print(nextState);
        Serial.print(" (Distance: ");
        Serial.print(currentDistance);
        Serial.println(" cm)");
        
        currentState = nextState;
        steps++;
        
        // Update LCD
        lcd.setCursor(0, 1);
        lcd.print("Step:");
        lcd.print(steps);
        lcd.print(" D:");
        lcd.print((int)currentDistance);
        lcd.print("cm");
        
        delay(500);
    }
    
    float endDistance = getDistance();
    totalDistance = startDistance - endDistance;
    
    Serial.print("\nTotal distance moved: ");
    Serial.print(totalDistance);
    Serial.println(" cm");
    Serial.print("Steps taken: ");
    Serial.println(steps);
    
    lcd.clear();
    lcd.print("Complete!");
    lcd.setCursor(0, 1);
    lcd.print("Moved: ");
    lcd.print((int)totalDistance);
    lcd.print(" cm");
    delay(3000);
}

void setup()
{
    Serial.begin(9600);
    delay(1000); // Give Serial Monitor time to connect

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);

    // Read or set robot number
    robot_number = readRobotNumber();
    snprintf(ssid, sizeof(ssid), "%s%d", base_ssid, robot_number);
    snprintf(ota_hostname, sizeof(ota_hostname), "%s%d", base_ota_hostname, robot_number);

    // Initialize AP and OTA
    setup_ap();
    setup_ota();

    // Start OTA task
    xTaskCreatePinnedToCore(
        otaTask,        // Task function
        "OTATask",      // Task name
        4096,           // Stack size
        NULL,           // Task parameters
        1,              // Priority
        &otaTaskHandle, // Task handle
        1               // Run on core 1
    );

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("RL Robot ");
    lcd.print(robot_number);
    lcd.setCursor(0, 1);
    lcd.print("Setup");
    delay(1000);

    // Attach servos with min/max pulse widths
    servodown.attach(control_servo_down, 600, 2400);
    servoup.attach(control_servo_up, 600, 2400);

    // Set servos to initial position
    servodown.write(0);
    servoup.write(180);
    servo1Angle = 0;
    servo2Angle = 180;

    // Initialize Q-Learning
    initializeQLearning();
    
    // Get initial distance
    previousDistance = getDistance();
    if (previousDistance > 0) {
        Serial.print("Initial distance: ");
        Serial.print(previousDistance);
        Serial.println(" cm");
    }

    // Notify setup completion
    lcd.clear();
    lcd.print("Setup Completed");
    delay(2000);
    healthCheck();
}

void loop()
{
    static bool trainingDone = false;
    static bool behaviorDone = false;
    static unsigned long lastActionTime = 0;
    
    if (!trainingDone) {
        lcd.clear();
        lcd.print("Robot ");
        lcd.print(robot_number);
        lcd.setCursor(0, 1);
        lcd.print("Main Loop");
        delay(1000); // Reduced delay to minimize blocking
        
        lcd.clear();
        lcd.print("Press to Train");
        lcd.setCursor(0, 1);
        lcd.print("Wait 5 sec...");
        
        delay(5000);  // Wait 5 seconds before starting training
        
        doTraining();
        trainingDone = true;
        delay(2000);
    } else if (!behaviorDone) {
        lcd.clear();
        lcd.print("Demo Learned");
        lcd.setCursor(0, 1);
        lcd.print("Behavior...");
        delay(2000);
        
        // Demonstrate learned behavior 3 times
        for (int demo = 0; demo < 3; demo++) {
            lcd.clear();
            lcd.print("Demo ");
            lcd.print(demo + 1);
            lcd.print(" of 3");
            delay(1000);
            
            doLearnedBehavior();
            delay(2000);
        }
        
        behaviorDone = true;
    } else {
        // Idle state - display status
        if (millis() - lastActionTime > 5000) {
            lcd.clear();
            lcd.print("Robot ");
            lcd.print(robot_number);
            lcd.setCursor(0, 1);
            lcd.print("Main Loop");
            delay(1000); // Reduced delay to minimize blocking
            
            lcd.clear();
            lcd.print("Robot ");
            lcd.print(robot_number);
            lcd.print(" Ready");
            lcd.setCursor(0, 1);
            lcd.print("Learned: ");
            lcd.print(successfulMoves);
            lcd.print(" moves");
            
            lastActionTime = millis();
        }
        
        // You can add serial commands here to retrain or demonstrate
        if (Serial.available()) {
            char cmd = Serial.read();
            if (cmd == 'r' || cmd == 'R') {
                trainingDone = false;
                behaviorDone = false;
                initializeQLearning();
                successfulMoves = 0;
                totalReward = 0;
                Serial.println("Restarting training...");
            } else if (cmd == 'd' || cmd == 'D') {
                doLearnedBehavior();
            }
        }
    }
    // TODO: Implement your main loop logic here
    
    delay(100);
}
