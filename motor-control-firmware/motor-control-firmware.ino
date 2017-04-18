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
double setpoint, input, output, signed_setpoint;
double k_p = 0.4;
double k_i = 0.0001;
double k_d = 0;
PID motor_pid_1(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

//---velocity test
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
double count = 0;
double velocity;
int interval = 50;
int ppr = 2000;

//---speed test
//const int analogInPin = 15;  // Analog input pin that the potentiometer is attached to
//const int analogOutPin = 5; // Analog output pin that the LED is attached to
//
//int sensorValue = 0;        // value read from the pot
//int outputValue = 0;        // value output to the PWM (analog out)

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

  analogWriteFrequency(5, 187500);

  //---PID
  motor_pid_1.SetMode(AUTOMATIC);
  motor_pid_1.SetSampleTime(10);
  setpoint = 3000;
  signed_setpoint = 3000;

  //---pot input
  // initialize all the readings to 0:
//  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
//    readings[thisReading] = 0;
//  }

  
}

void loop() {
  //CW  (1) encoder is positive
  //CCW (2) encoder is negative

  //---smoothing the variable setpoint
    // subtract the last reading:
//    total = total - readings[readIndex];
//    // read from the sensor:
//    readings[readIndex] = analogRead(analogInPin);
//    // add the reading to the total:
//    total = total + readings[readIndex];
//    // advance to the next position in the array:
//    readIndex = readIndex + 1;
//    // if we're at the end of the array...
//    if (readIndex >= numReadings) {
//      // ...wrap around to the beginning:
//      readIndex = 0;
//    }
//    // calculate the average:
//    sensorValue = total / numReadings;
//  
//    setpoint = map(sensorValue, 0, 1023, -10000, 10000);

//---get setpoint from serial port

  if (Serial.available() > 0) {
    // read the incoming byte:
    incoming_data = Serial.parseInt();
  }

  setpoint = incoming_data;
  
  newtime = millis();

  if(newtime - oldtime >= interval){
  
    oldtime = newtime; //set the old time
    newposition = enc_1.read(); //find the new position
    count = newposition-oldposition; //find the count since the last interval
    velocity = ((count/ppr)*60)/(interval/1000); //calculate velocity
    oldposition = newposition; //set the old position
  }
  
  
  input = velocity;  
  motor_pid_1.Compute();
  set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, output);


//---show results (plotter)
  Serial.print(input);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.print(signed_setpoint);
  delay(1);

//  Serial.print("Current position = ");
//  Serial.print(input);
//  Serial.print("\t Desired position = ");
//  Serial.print(setpoint);
//  Serial.print("\t PWM output = ");
//  Serial.println(out);
//  delay(2);

}




