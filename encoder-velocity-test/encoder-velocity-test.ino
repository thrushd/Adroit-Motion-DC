#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Encoder.h>
#include <Plotter.h>

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

//---velocity test
long newposition;
long oldposition = 0;
double count = 0;
double interval = 50;
double velocity = 0;
int ppr = 2000; //pulses per rev
unsigned long newtime;
unsigned long oldtime = 0;

const int analogInPin = 15;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 5; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)


//---filtering
const int numReadings = 100;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

//---PID
double setpoint, input, output, signed_setpoint;
double k_p = 0.2;
double k_i = 1;
double k_d = 0.0005;

PID motor_pid_1(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

//---debug

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
  setpoint = 60;

//---filtering
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {

//---read and map new speed
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
  
    signed_setpoint = map(sensorValue, 0, 1023, -3000, 3000);
 
//---calculate velocity
  newtime = millis();

  if(newtime - oldtime >= interval){
  
    oldtime = newtime; //set the old time
    newposition = enc_1.read(); //find the new position
    count = newposition-oldposition; //find the count since the last interval
    velocity = ((count/ppr)*60)/(interval/1000); //calculate velocity
    oldposition = newposition; //set the old position
  }
  
//---do PID stuff

  if(signed_setpoint > 0){ //positive
    input = velocity;
    setpoint = signed_setpoint;
    motor_pid_1.Compute();
    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, output);
    //Serial.println("Positive");
  }
  else { //negative
    input = abs(velocity);
    setpoint = abs(signed_setpoint);
    motor_pid_1.Compute();
    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, output);
    //Serial.println("Negative");
  }

//---show results (plotter)
  Serial.print(input);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(signed_setpoint);
  Serial.print(",");
  Serial.println(setpoint);
  delay(1);
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
