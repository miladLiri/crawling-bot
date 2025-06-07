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

void doTraining()
{
    // TODO: Add your training logic here
}

void doLearnedBehavior()
{
    // TODO: Add your learned behavior logic here
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
}

void loop()
{
    lcd.clear();
    lcd.print("Robot ");
    lcd.print(robot_number);
    lcd.setCursor(0, 1);
    lcd.print("Main Loop");
    delay(1000); // Reduced delay to minimize blocking
    // TODO: Implement your main loop logic here
}