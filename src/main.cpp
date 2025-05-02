#include <Arduino.h>
#include <Servo.h>

#define SOUND_PIN 13
#define LIGHT_PIN A0
#define SERVO_PIN 2

Servo servo;

// put function declarations here:
// int myFunction(int, int);

void setup()
{
  Serial.begin(9600);
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  pinMode(SOUND_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);
  // pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  servo.write(0);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Serial.println(digitalRead(SOUND_PIN));
  // delay(100);
  // servo.write(0);


  // servo.write(100);
  // delay(1000);

  if(digitalRead(SOUND_PIN)) {
    servo.write(30);
    delay(100);
    servo.write(0);
    delay(1000);
  }

  Serial.println(analogRead(LIGHT_PIN));
  delay(10);
}

// put function definitions here:
// int myFunction(int x, int y) {
//   return x + y;
// }