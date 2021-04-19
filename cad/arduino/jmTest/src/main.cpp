#include <Arduino.h>
#include <Servo.h>  // add servo library

#define STATUS_PIN LED_BUILTIN
#define SERVO_PIN 9

Servo myservo;

int potpin = 7;
int val;
char sendVal[20];

enum status{
  cycle_on, cycle_off, spray_control 
};

status state = cycle_on;

int val_on = 50;
int val_off = 105;

int off_time = 1000; 
int on_time = 3000;

void setup() {
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, LOW);
  myservo.attach(SERVO_PIN);

  Serial.begin(9600);
  delay(10);

  if (state == cycle_on){
    myservo.write(val_off);
    Serial.write("off\n");
    delay(off_time);
    myservo.write(val_on);
    Serial.write("on\n");
    delay(on_time);
    myservo.write(val_off);
    Serial.write("forever off\n\n");
    digitalWrite(STATUS_PIN, HIGH);
    state = cycle_off;
    delay(10000);
  }
}

void loop() {
  //blocking for now
  if (state == spray_control){
    val = analogRead(potpin);            
    val = map(val, 0, 1023, 0, 180);     

    itoa(val, sendVal, 10);
    Serial.write(sendVal);
    //Serial.write(val);
    Serial.write('\n');
    myservo.write(val);
    delay(15);                
  }           
}
