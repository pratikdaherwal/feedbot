#include <ESP8266WiFi.h>
#include <AccelStepper.h>

// Wi-Fi credentials
const char* ssid = "Your_SSID";
const char* password = "Your_PASSWORD";

// Pin definitions
#define in1 5   // GPIO5 (D1 on NodeMCU)
#define in2 4   // GPIO4 (D2 on NodeMCU)
#define in3 0   // GPIO0 (D3 on NodeMCU)
#define in4 2   // GPIO2 (D4 on NodeMCU)
#define enA 14  // GPIO14 (D5 on NodeMCU)
#define enB 12  // GPIO12 (D6 on NodeMCU)
#define LEFT_SENSOR 15  // Analog pin A0 for the Left Sensor
#define RIGHT_SENSOR 13 // GPIO13 (D7) for the Right Sensor (Digital Pin)
#define STEP_PIN 16     // GPIO16 for Step pin of A4988
#define DIR_PIN 2       // GPIO2 for Direction pin of A4988
#define RELAY_PIN 10    // GPIO10 for Relay to control conveyor belt motor

// Motor speed variables
int M1_Speed = 400; // speed of motor 1
int M2_Speed = 400; // speed of motor 2
int LeftRotationSpeed = 800;  // Left Rotation Speed
int RightRotationSpeed = 800; // Right Rotation Speed

// Stepper motor configuration
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Pin setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Stepper motor setup
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  // Ensure relay is off initially
  digitalWrite(RELAY_PIN, LOW);
}

void loop() {
  int LEFT_SENSOR_STATE = digitalRead(LEFT_SENSOR);
  int RIGHT_SENSOR_STATE = digitalRead(RIGHT_SENSOR);

  if (RIGHT_SENSOR_STATE == 0 && LEFT_SENSOR_STATE == 0) {
    forward(); // FORWARD
  } else if (RIGHT_SENSOR_STATE == 0 && LEFT_SENSOR_STATE == 1) {
    right(); // Move Right
  } else if (RIGHT_SENSOR_STATE == 1 && LEFT_SENSOR_STATE == 0) {
    left(); // Move Left
  } else if (RIGHT_SENSOR_STATE == 1 && LEFT_SENSOR_STATE == 1) {
    Stop(); // STOP and execute conveyor task
    rotateStepperAndConveyor();
  }
}

void forward() {
  Serial.println("FORWARD");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void backward() {
  Serial.println("BACKWARD");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, M1_Speed);
  analogWrite(enB, M2_Speed);
}

void right() {
  Serial.println("RIGHT");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void left() {
  Serial.println("LEFT");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, LeftRotationSpeed);
  analogWrite(enB, RightRotationSpeed);
}

void Stop() {
  Serial.println("STOP");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void rotateStepperAndConveyor() {
  Serial.println("Rotating stepper and starting conveyor");

  // Rotate stepper motor 180 degrees
  stepper.moveTo(200); // Adjust for your stepper motor's steps per revolution
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Start conveyor belt using relay
  digitalWrite(RELAY_PIN, HIGH);
  delay(5000); // Conveyor belt runs for 5 seconds (adjust as needed)

  // Stop conveyor belt
  digitalWrite(RELAY_PIN, LOW);
  delay(1000);

  // Rotate stepper motor back to initial position
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  Serial.println("Stepper motor and conveyor task completed");
}
