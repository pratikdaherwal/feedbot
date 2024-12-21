/*
Code Name: NodeMCU Line Follower Robot Car
Author: Adapted for NodeMCU
Description: Line Follower Robot Car using NodeMCU with digital sensors.
*/

#define in1 5   // GPIO5 (D1 on NodeMCU)
#define in2 4   // GPIO4 (D2 on NodeMCU)
#define in3 0   // GPIO0 (D3 on NodeMCU)
#define in4 2   // GPIO2 (D4 on NodeMCU)
#define enA 14  // GPIO14 (D5 on NodeMCU)
#define enB 12  // GPIO12 (D6 on NodeMCU)

#define LEFT_SENSOR 15  // Analog pin A0 for the Left Sensor
#define RIGHT_SENSOR 13 // GPIO13 (D7) for the Right Sensor (Digital Pin)

int M1_Speed = 400; // speed of motor 1
int M2_Speed = 400; // speed of motor 2
int LeftRotationSpeed = 800;  // Left Rotation Speed
int RightRotationSpeed = 800; // Right Rotation Speed

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(LEFT_SENSOR, INPUT);  // Left sensor as digital input
  pinMode(RIGHT_SENSOR, INPUT); // Right sensor as digital input
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
    Stop(); // STOP
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
  Serial.println("STOP");
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
