#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Encoder.h>
#include <Plotter.h>

#define BRAKEVCC  0
#define CW        1
#define CCW       2
#define BRAKEGND  3

/*
  pin definitions:
  xxx[0] controls '1' outputs
  xxx[1] controls '2' outputs */
int inApin[2] = {27, 30};  // INA: Clockwise input
int inBpin[2] = {28, 29}; // INB: Counter-clockwise input
int pwmpin[2] = {25, 32}; // PWM input
int cspin[2] = {31, 26}; // CS: Current sense ANALOG input
int enpin[2] = {24, 33}; // EN: Status of switches output (Analog pin)

//---encoders
#define enc_pin_1 14
#define enc_pin_2 3
long old_position = -999;
Encoder enc_1(enc_pin_1, enc_pin_2);

//---velocity test
long newposition;
long oldposition = 0;
double count = 0;
double interval = 2;
double velocity = 0;
int ppr = 2000; //pulses per rev
unsigned long newtime;
unsigned long oldtime = 0;

#define incr 6
#define decr 7
long move_amount = 10; //RPM

//---PID
double setpoint, input, output, signed_setpoint;
double k_p = 2;
double k_i = 0;
double k_d = 0;

PID motor_pid_1(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

//---debug

void setup() {
  //---debug
  Serial.begin(9600);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  // motorGo(0, CW, 1023);
  // motorGo(1, CCW, 1023);

  //set PWM frequency
  analogWriteFrequency(pwmpin[0], 187500);
  analogWriteFrequency(pwmpin[1], 187500);

  //---PID
  motor_pid_1.SetMode(AUTOMATIC);
  motor_pid_1.SetSampleTime(1);
  setpoint = 0;
  signed_setpoint = 0;

  //---buttons
  pinMode(incr, INPUT);
  pinMode(decr, INPUT);
  digitalWrite(incr, INPUT_PULLUP);
  digitalWrite(decr, INPUT_PULLUP);
}


void loop() {

  //---use buttons to increment or decrement the position
  if (digitalRead(incr) == LOW) {
    signed_setpoint += move_amount;
    Serial.println("Increment!");
    delay(200);
  }
  else if (digitalRead(decr) == LOW) {
    signed_setpoint -= move_amount;
    Serial.println("Decrement!");
    delay(200);
  }

  set_velocity(0, signed_setpoint);

  //---show results (plotter)
  Serial.print(input);
  Serial.print(",");
  Serial.print(output);
  Serial.print(",");
  Serial.println(signed_setpoint);
  //delay(1);
}

/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either 0 or 1, will selet which of the two
  motors to be controlled

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
*/
void set_motor_output(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void set_velocity(int motor_id, double velocity) {
  //---calculate velocity
  newtime = millis();

  if (newtime - oldtime >= interval) {

    oldtime = newtime; //set the old time
    newposition = enc_1.read(); //find the new position
    count = newposition - oldposition; //find the count since the last interval
    velocity = ((count / ppr) * 60) / (interval / 1000); //calculate velocity
    oldposition = newposition; //set the old position
  }

  //  //---do PID stuff
  if (signed_setpoint > 0) { //positive
    input = velocity;
    setpoint = signed_setpoint;
    motor_pid_1.Compute();
    set_motor_output(motor_id, 1, output);
    //Serial.println("Positive");
  }
  else { //negative
    input = abs(velocity);
    setpoint = abs(signed_setpoint);
    motor_pid_1.Compute();
    set_motor_output(motor_id, 2, output);
    //Serial.println("Negative");
  }

}

