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
#define EEPROM_SIZE 512    // Increased for Q-table storage
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

// Q-Learning Parameters
#define NUM_STATES 8         // Distance ranges
#define NUM_ACTIONS 6        // Different crawling patterns
#define ALPHA 0.3           // Learning rate
#define GAMMA 0.9           // Discount factor
#define INITIAL_EPSILON 0.9 // Initial exploration rate
#define MIN_EPSILON 0.1     // Minimum exploration rate
#define EPSILON_DECAY 0.995 // Epsilon decay rate
#define Q_TABLE_ADDR 10     // EEPROM address for Q-table

// Q-Learning variables
float qTable[NUM_STATES][NUM_ACTIONS];
float epsilon = INITIAL_EPSILON;
int currentState = 0;
int previousState = 0;
int currentAction = 0;
float previousDistance = 0;
int episodeCount = 0;
int stepCount = 0;
bool isTraining = true;

// Action definitions (servo movement patterns)
struct Action {
    int servoDownStart;
    int servoDownEnd;
    int servoUpStart;
    int servoUpEnd;
    int duration;
    const char* name;
};

// Define 6 different crawling patterns
Action actions[NUM_ACTIONS] = {
    {0, 45, 180, 135, 500, "Forward Push"},
    {45, 0, 135, 180, 500, "Forward Pull"},
    {0, 90, 180, 90, 600, "Wide Push"},
    {90, 0, 90, 180, 600, "Wide Pull"},
    {0, 30, 150, 180, 400, "Quick Push"},
    {30, 0, 180, 150, 400, "Quick Pull"}
};

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
        return MAX_DISTANCE; // Return max distance instead of -1 for Q-learning
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

// Q-Learning Helper Functions
void initQTable() {
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_ACTIONS; j++) {
            qTable[i][j] = (float)random(0, 10) / 100.0;
        }
    }
}

void saveQTable() {
    int addr = Q_TABLE_ADDR;
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_ACTIONS; j++) {
            EEPROM.writeFloat(addr, qTable[i][j]);
            addr += sizeof(float);
        }
    }
    EEPROM.commit();
    Serial.println("Q-table saved to EEPROM");
}

void loadQTable() {
    int addr = Q_TABLE_ADDR;
    bool hasValidData = false;
    
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_ACTIONS; j++) {
            float value = EEPROM.readFloat(addr);
            if (!isnan(value) && value != 0) {
                hasValidData = true;
                qTable[i][j] = value;
            } else {
                qTable[i][j] = (float)random(0, 10) / 100.0;
            }
            addr += sizeof(float);
        }
    }
    
    if (hasValidData) {
        Serial.println("Q-table loaded from EEPROM");
    } else {
        Serial.println("No valid Q-table found, initialized new table");
    }
}

int getState(float distance) {
    if (distance < 25) return 0;
    else if (distance < 50) return 1;
    else if (distance < 75) return 2;
    else if (distance < 100) return 3;
    else if (distance < 125) return 4;
    else if (distance < 150) return 5;
    else if (distance < 175) return 6;
    else return 7;
}

int selectAction(int state) {
    if ((float)random(0, 100) / 100.0 < epsilon) {
        // Exploration: random action
        return random(0, NUM_ACTIONS);
    } else {
        // Exploitation: best action from Q-table
        int bestAction = 0;
        float maxQ = qTable[state][0];
        
        for (int i = 1; i < NUM_ACTIONS; i++) {
            if (qTable[state][i] > maxQ) {
                maxQ = qTable[state][i];
                bestAction = i;
            }
        }
        return bestAction;
    }
}

void executeAction(int action) {
    Action& act = actions[action];
    
    Serial.print("Executing action: ");
    Serial.println(act.name);
    
    // Execute the crawling pattern
    moveServoSmooth(servodown, servodown.read(), act.servoDownStart, 5);
    moveServoSmooth(servoup, servoup.read(), act.servoUpStart, 5);
    delay(act.duration / 2);
    
    moveServoSmooth(servodown, act.servoDownStart, act.servoDownEnd, 5);
    moveServoSmooth(servoup, act.servoUpStart, act.servoUpEnd, 5);
    delay(act.duration / 2);
    
    // Return to neutral position
    moveServoSmooth(servodown, act.servoDownEnd, 0, 5);
    moveServoSmooth(servoup, act.servoUpEnd, 180, 5);
}

float calculateReward(float oldDistance, float newDistance) {
    float reward = 0;
    float distanceChange = newDistance - oldDistance;
    
    if (distanceChange > 0) {
        reward = distanceChange / 10.0;  // Positive reward
        if (newDistance > 100) {
            reward += 0.5;  // Bonus for safe distance
        }
    } else if (distanceChange < 0) {
        reward = distanceChange / 5.0;  // Penalty for getting closer
        if (newDistance < 30) {
            reward -= 1.0;  // Extra penalty for too close
        }
    } else {
        reward = -0.1;  // Small penalty for no progress
    }
    
    return reward;
}

void updateQValue(int state, int action, float reward, int nextState) {
    float maxNextQ = qTable[nextState][0];
    for (int i = 1; i < NUM_ACTIONS; i++) {
        if (qTable[nextState][i] > maxNextQ) {
            maxNextQ = qTable[nextState][i];
        }
    }
    
    // Q-learning update rule
    qTable[state][action] = qTable[state][action] + 
                            ALPHA * (reward + GAMMA * maxNextQ - qTable[state][action]);
    
    Serial.print("Updated Q[");
    Serial.print(state);
    Serial.print("][");
    Serial.print(action);
    Serial.print("] = ");
    Serial.println(qTable[state][action]);
}

void doTraining()
{
    const int STEPS_PER_EPISODE = 20;
    
    // Initialize episode if needed
    if (stepCount == 0) {
        previousDistance = getDistance();
        previousState = getState(previousDistance);
        Serial.print("Episode ");
        Serial.print(episodeCount);
        Serial.println(" started");
    }
    
    // Training step
    stepCount++;
    
    // Select and execute action
    currentAction = selectAction(previousState);
    executeAction(currentAction);
    
    // Get new distance and state
    delay(500);
    float currentDistance = getDistance();
    currentState = getState(currentDistance);
    
    // Calculate reward
    float reward = calculateReward(previousDistance, currentDistance);
    
    // Update Q-value
    updateQValue(previousState, currentAction, reward, currentState);
    
    // Display progress
    lcd.clear();
    lcd.print("Train E:");
    lcd.print(episodeCount);
    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print((int)currentDistance);
    lcd.print(" R:");
    lcd.print(reward, 1);
    
    // Prepare for next step
    previousState = currentState;
    previousDistance = currentDistance;
    
    // Check if episode is complete
    if (stepCount >= STEPS_PER_EPISODE) {
        stepCount = 0;
        episodeCount++;
        
        // Decay epsilon
        epsilon = max(MIN_EPSILON, epsilon * EPSILON_DECAY);
        
        // Save Q-table every 10 episodes
        if (episodeCount % 10 == 0) {
            saveQTable();
        }
        
        Serial.print("Episode ");
        Serial.print(episodeCount - 1);
        Serial.println(" complete");
    }
}

void doLearnedBehavior()
{
    // Get current distance and state
    float currentDistance = getDistance();
    int state = getState(currentDistance);
    
    // Select best action (no exploration)
    int bestAction = 0;
    float maxQ = qTable[state][0];
    
    for (int i = 1; i < NUM_ACTIONS; i++) {
        if (qTable[state][i] > maxQ) {
            maxQ = qTable[state][i];
            bestAction = i;
        }
    }
    
    // Display status
    lcd.clear();
    lcd.print("Learned Mode");
    lcd.setCursor(0, 1);
    lcd.print("D:");
    lcd.print((int)currentDistance);
    lcd.print(" A:");
    lcd.print(bestAction);
    
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.print(" State: ");
    Serial.print(state);
    Serial.print(" Action: ");
    Serial.println(actions[bestAction].name);
    
    // Execute best action
    executeAction(bestAction);
    
    delay(1000);
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

    // Notify setup completion
    lcd.clear();
    lcd.print("Setup Completed");
    delay(2000);
    healthCheck();
    
    // Initialize Q-Learning
    randomSeed(analogRead(0));
    loadQTable();
    
    // Ask user for mode via Serial
    lcd.clear();
    lcd.print("Mode Select:");
    lcd.setCursor(0, 1);
    lcd.print("1:Train 2:Run");
    
    Serial.println("Select Mode: 1=Training, 2=Learned Behavior");
    unsigned long startTime = millis();
    while (!Serial.available() && (millis() - startTime < 5000)) {
        delay(100);
    }
    
    if (Serial.available()) {
        int mode = Serial.parseInt();
        if (mode == 2) {
            isTraining = false;
            epsilon = 0;
            Serial.println("Starting in Learned Behavior Mode");
            lcd.clear();
            lcd.print("Learned Mode");
            delay(2000);
        } else {
            Serial.println("Starting in Training Mode");
            lcd.clear();
            lcd.print("Training Mode");
            delay(2000);
        }
    } else {
        Serial.println("No input - defaulting to Training Mode");
        lcd.clear();
        lcd.print("Training Mode");
        lcd.print(" (Default)");
        delay(2000);
    }
}

void loop()
{
    if (isTraining) {
        doTraining();
    } else {
        doLearnedBehavior();
    }
    delay(100);
}
