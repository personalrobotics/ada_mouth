#include <Servo.h>

#define MAX_SERVO_CMD 90 // The max value servo cmd accepted
#define MIN_SERVO_CMD 20 // The min value servo cmd accepted

#define SERIAL_BAUD_RATE 9600 // Serial communication speed
#define SERVO_PIN 9 // Pin number of output to servo

#define TRANSFER_RATE 20 // Rate at which to output strain gauge data
#define SENSOR_COUNT 4  // Number of strain gauge sensors
#define LEFT_SENSOR_PIN A0 // Pin for sensor on left side of mouth
#define RIGHT_SENSOR_PIN A1 // Pin for sensor on right side of mouth
#define TOP_SENSOR_PIN A2 // Pin for sensor on roof of mouth
#define BOTTOM_SENSOR_PIN A3 // Pin for sensor on floor of mouth

Servo servo;
unsigned long last_sample = 0; // Timestamp of previous sensor sample sent
uint8_t sensor_buff[2*SENSOR_COUNT]; // Write buffer for sensor data
unsigned int led_count = 0;

// Initialize the mouth
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  servo.attach(SERVO_PIN);

  delay(10);

  // Open the mouth by default
  servo.write(MIN_SERVO_CMD);
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  // Process servo commands
  if(Serial.available() > 0) { // Check if cmd has come in
    int servo_cmd = Serial.read(); // Read the cmd
    
    // Check if cmd is in range
    if(servo_cmd >= MIN_SERVO_CMD && servo_cmd <= MAX_SERVO_CMD) {
      servo.write(servo_cmd); // Send cmd to servo
    }
  }
  
  // Read sensor vals from strain gauge boards
  int left_sensor_val = analogRead(LEFT_SENSOR_PIN);
  int right_sensor_val = analogRead(RIGHT_SENSOR_PIN);
  int top_sensor_val = analogRead(TOP_SENSOR_PIN);
  int bottom_sensor_val = analogRead(BOTTOM_SENSOR_PIN);

  unsigned long now = millis();
  float elapsed = (now - last_sample) / 1000.0;
  // Check if it is time to send data
  if(elapsed > 1.0/TRANSFER_RATE) {
    last_sample = now;

    // Each strain gauge reading is serialized 
    // into two bytes, where the 5 lsb of each
    // byte contain the data
    sensor_buff[0] = left_sensor_val & 0x01F;
    sensor_buff[1] = left_sensor_val >> 5;
    sensor_buff[2] = right_sensor_val & 0x01F;
    sensor_buff[3] = right_sensor_val >> 5;
    sensor_buff[4] = top_sensor_val & 0x01F;
    sensor_buff[5] = top_sensor_val >> 5;
    sensor_buff[6] = bottom_sensor_val & 0x01F;
    sensor_buff[7] = bottom_sensor_val >> 5;   

    // Add an id to the 3 msb of each byte
    for(int i = 0; i < 2*SENSOR_COUNT; i++) {
      sensor_buff[i] |= (i << 5); 
    }

    // Send the data to host         
    Serial.write(sensor_buff, 2*SENSOR_COUNT);

    if(led_count < TRANSFER_RATE/2)
    {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
    led_count = (led_count + 1) % TRANSFER_RATE;
  }
  
}
