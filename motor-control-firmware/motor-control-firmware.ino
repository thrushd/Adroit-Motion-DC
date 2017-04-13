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
double k_p = 3.2;
double k_i = 12.8;
double k_d = 0.01;
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

void setup() {
//---debug
Serial.begin(115200);

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
}

void loop() {

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

void set_motor_output(uint8_t pin_a, uint8_t pin_b, uint8_t pwm_pin, uint8_t direct, uint8_t pwm){

    if (direct <=4)
    {
      // Set A pin
      if (direct <=1)
        digitalWrite(pin_a, HIGH); //0, 1
      else
        digitalWrite(pin_a, LOW); //2, 3

      // Set B pin
      if ((direct==0)||(direct==2))
        digitalWrite(pin_b, HIGH); //0, 2
      else
        digitalWrite(pin_b, LOW); //1, 3

      analogWrite(pwm_pin, pwm);
    }
}


