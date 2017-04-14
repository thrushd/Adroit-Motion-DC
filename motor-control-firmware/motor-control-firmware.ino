#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Encoder.h>

//---motors
#define in_a_1    7
#define in_b_1    8
#define pwm_pin_1 5
#define cs_pin_1  14
#define en_pin_1  4

#define BRAKEVCC  0
#define CW        1
#define CCW       2
#define BRAKEGND  3

//---encoders
#define enc_pin_1 2
#define enc_pin_2 3
long old_position = -999;
Encoder enc_1(enc_pin_1, enc_pin_2);

//---PID
double setpoint, input, out;
double k_p = 0.4;
double k_i = 0.0005;
double k_d = 0;
PID motor_pid_1(&input, &out, &setpoint, k_p, k_i, k_d, DIRECT);

//---velocity test
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
long vel;

//---speed test
const int analogInPin = 15;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 5; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

//---position test
const int numReadings = 100;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

//---serial testing
int incoming_data = 0; // for incoming serial data

void setup() {
  //---debug
  Serial.begin(9600);

  //---motors
  pinMode(in_a_1, OUTPUT);
  pinMode(in_b_1, OUTPUT);
  pinMode(pwm_pin_1, OUTPUT);
  pinMode(cs_pin_1, OUTPUT);
  pinMode(en_pin_1, OUTPUT);
  digitalWrite(en_pin_1, HIGH);

  //---PID
  motor_pid_1.SetMode(AUTOMATIC);
  motor_pid_1.SetSampleTime(10);
  //setpoint = 50000;

  //---position testing
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  //CW  (1) encoder is positive
  //CCW (2) encoder is negative

  //---smoothing the variable setpoint
    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = analogRead(analogInPin);
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;
    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }
    // calculate the average:
    sensorValue = total / numReadings;
  
    setpoint = map(sensorValue, 0, 1023, -10000, 10000);

  //---get setpoint from serial port

//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incoming_data = Serial.parseInt();
//
//    // say what you got:
//    Serial.print("I received: ");
//    Serial.println(incoming_data, DEC);
//  }
//
//  setpoint = incoming_data;
  
  input = enc_1.read();

  if (input > setpoint) { //positive
    motor_pid_1.SetControllerDirection(REVERSE);
    motor_pid_1.Compute();
    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, out);
  }
  else { //negative
    motor_pid_1.SetControllerDirection(DIRECT);
    motor_pid_1.Compute();
    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, out);
  }

  Serial.print("Current position = ");
  Serial.print(input);
  Serial.print("\t Desired position = ");
  Serial.print(setpoint);
  Serial.print("\t PWM output = ");
  Serial.println(out);

  delay(2);

  /*---------------speed control----------------------------
    //---read new speed
    sensorValue = analogRead(analogInPin);
    // map it to the range of the analog out:
    setpoint = map(sensorValue, 0, 1023, 0, 100);

    //---calculate velocity
    newposition = enc_1.read();
    newtime = millis();
    vel = (newposition-oldposition)/(newtime-oldtime);
    oldposition = newposition;
    oldtime = newtime;

    //---PID
    input = vel;
    motor_pid_1.Compute();

    //---set outputs
    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, out);

    //---show results
    Serial.print("Current speed = ");
    Serial.print(input);
    Serial.print("\t Desired speed = ");
    Serial.print(setpoint);
    Serial.print("\t PWM output = ");
    Serial.print(out);
    Serial.print("\t Raw input = ");
    Serial.println(sensorValue);

    delay(5);
    ---------------------------------------------------------*/
}


/*
  Will set a motor going in a specific direction the motor will continue
  going in that direction, at that speed until told to do otherwise.

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
  ----------------
  Control Logic:
  ----------------
           A | B
  Brake VCC: 1   1
       CW: 1   0
      CCW: 0   1
  Brake GND: 0   0
  ----------------
*/

void set_motor_output(uint8_t pin_a, uint8_t pin_b, uint8_t pwm_pin, uint8_t direct, uint8_t pwm) {

  if (direct <= 4)
  {
    // Set A pin
    if (direct <= 1)
      digitalWrite(pin_a, HIGH); //0, 1
    else
      digitalWrite(pin_a, LOW); //2, 3

    // Set B pin
    if ((direct == 0) || (direct == 2))
      digitalWrite(pin_b, HIGH); //0, 2
    else
      digitalWrite(pin_b, LOW); //1, 3

    analogWrite(pwm_pin, pwm);
  }
}


