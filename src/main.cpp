#include <Arduino.h>
#include <Servo.h>

#define SOUND_PIN 13
#define LIGHT_PIN A0
#define SERVO_PIN 12

const String COMMAND_PREF = "ardmon";

Servo servo;

/*
Writes into serial using prefixes:
  - ":" prefix is a log
  - "=" prefix is an error

  its logs are ":sound-detected,light-level" type: digital,analog
  its errors are "=error-message" type: string
*/

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
  delay(10);
}

void logData() {
  // log: ":sound-detected,light-level" type: digital,analog
  Serial.println(":" + String(digitalRead(SOUND_PIN)) + "," + String(analogRead(LIGHT_PIN)));
  if (Serial.available()) {
    Serial.println(Serial.read());
  }
}

void parseCommand() {
  int serial_index = 0;
  bool contain_prefix = false; // if command starts with "ardmon" set true

  const int MAX_FLAG_LENGTH = 64;
  char flag[MAX_FLAG_LENGTH];
  int flag_index = 0;

  while(Serial.available()) {
    char c = Serial.read();

    // terminate on line break
    if (c == '\n') {
      return;
    }

    // reading command flags
    if (contain_prefix) {
      
      // if space or max index, process current flag
      if (c == ' ' || flag_index < MAX_FLAG_LENGTH - 1) {
        flag[flag_index] = '\0'; // terminate the string
        flag_index = 0;
        continue;
      }

      // build the flag as char[] into string
      flag[flag_index] = c;
      flag_index++;
      
      continue; // to skip the command prefix check
    }

    // check if line contains command prefix => read only if it is
    if (c == COMMAND_PREF[serial_index]) {
      if(serial_index + 1 < COMMAND_PREF.length()) {
        serial_index++;
        continue;
      }
      contain_prefix = true;
    }
    else {
      Serial.println("=Unknown Command");
      clearSerialBuffer();
      return;
    }

  }
}

void clearSerialBuffer() {
  while(Serial.available()) {
    Serial.read(); // read to remove from buffer
  }
}

void moveServo() {
  servo.write(30);
  delay(10);
  servo.write(0);
  delay(100);
}