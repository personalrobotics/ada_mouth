#include <Servo.h>

#define MAX_SERVO_CMD 90
#define MIN_SERVO_CMD 20

#define SERIAL_BAUD_RATE 9600
#define SERVO_PIN 9

#define TRANSFER_RATE 20
#define SENSOR_COUNT 4
#define LEFT_SENSOR_PIN A0
#define RIGHT_SENSOR_PIN A1
#define TOP_SENSOR_PIN A2
#define BOTTOM_SENSOR_PIN A3

Servo servo;
unsigned long last_sample = 0;
uint8_t sensor_buff[2*SENSOR_COUNT];

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  servo.attach(SERVO_PIN);

  delay(10);

  servo.write(MIN_SERVO_CMD);
  
}

void loop() {

  
  if(Serial.available() > 0) {
    int servo_cmd = Serial.read();
    if(servo_cmd >= MIN_SERVO_CMD && servo_cmd <= MAX_SERVO_CMD) {
      servo.write(servo_cmd);
    }
  }
  

  int left_sensor_val = analogRead(LEFT_SENSOR_PIN);
  int right_sensor_val = analogRead(RIGHT_SENSOR_PIN);
  int top_sensor_val = analogRead(TOP_SENSOR_PIN);
  int bottom_sensor_val = analogRead(BOTTOM_SENSOR_PIN);

  unsigned long now = millis();
  float elapsed = (now - last_sample) / 1000.0;
  if(elapsed > 1.0/TRANSFER_RATE) {
    last_sample = now;
    sensor_buff[0] = left_sensor_val & 0x01F;
    sensor_buff[1] = left_sensor_val >> 5;
    sensor_buff[2] = right_sensor_val & 0x01F;
    sensor_buff[3] = right_sensor_val >> 5;
    sensor_buff[4] = top_sensor_val & 0x01F;
    sensor_buff[5] = top_sensor_val >> 5;
    sensor_buff[6] = bottom_sensor_val & 0x01F;
    sensor_buff[7] = bottom_sensor_val >> 5;   

    for(int i = 0; i < 2*SENSOR_COUNT; i++) {
      sensor_buff[i] |= (i << 5); 
    }
             
    Serial.write(sensor_buff, 2*SENSOR_COUNT);
  }
  
}
