#include <Arduino.h>
#include <Servo.h>

#define SOUND_PIN 13
#define LIGHT_PIN A0
#define SERVO_PIN 12

Servo servo;

/*
Writes into serial using prefixes:
  - ":" prefix is a log
  - "=" prefix is an error

  its logs are ":sound-detected,light-level" type: digital,analog
  its errors are "=error-message" type: string
*/

void logData();
void moveServo();
void clearSerialBuffer();
void parseCommand();
void runCommand(String flag);

void setup()
{
  Serial.begin(9600);
  pinMode(SOUND_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(0);
}

void loop()
{
  logData();
  parseCommand();
  delay(50); // TODO: shorten delay
}

void logData() {
  // log: ":sound-detected,light-level" type: digital,analog
  Serial.println(":" + String(digitalRead(SOUND_PIN)) + "," + String(analogRead(LIGHT_PIN)));
}

void clearSerialBuffer() {
  while(Serial.available()) {
    Serial.read(); // read to remove from buffer
  }
}

void moveServo() {
  servo.write(30);
  delay(100);
  servo.write(0);
  delay(500);
}

void parseCommand() {
  const int MAX_FLAG_LENGTH = 64;
  char flag[MAX_FLAG_LENGTH];
  int flag_index = 0;

  while(Serial.available()) {
    delay(10);
    char c = Serial.read();

    // if space, line break, or max index, process current flag
    if (c == ' ' || c == '\n' || !(flag_index < MAX_FLAG_LENGTH - 1)) {
      flag[flag_index] = '\0'; // terminate the string

      runCommand(flag);

      flag_index = 0;
      continue;
    }

    // build the flag as char[] into string
    flag[flag_index] = c;
    flag_index++;
  }
}

void runCommand(String flag) {
  if (flag == "-ms") {
    moveServo();
  }
  else {
    Serial.println("=Unknown flag");
  }
}

