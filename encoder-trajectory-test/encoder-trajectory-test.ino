#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <Encoder.h>

//---motor stuff
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

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
long old_position = -999;
Encoder enc_1(14, 15);
int mult = 1110; //counts per mm for this encoder and linear state, axis R

//---PID
double setpoint, input, output;
double k_p = 0.5;
double k_i = 0;
double k_d = 0.003;
PID motor_pid_1(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);

//---status
int statpin = 13;

//---buttons test
#define incr 6
#define decr 7
long move_amount = 5000; //counts
double act_pos = 0; //

//---trajectory mapping
double acc = 200000;  //acceleration to use
double pos_curr = 0; //current position
double pos_final = 0; //desired position
double time_final = 0; //time to complete movement
double a[3]; //constants
double b[3];
unsigned long time_new; //used for timing
unsigned long time_old = 0;
double time_curr = 0;


//---serial testing
int incoming_data = 0; // for incoming serial data

int i = 1;

void setup()
{
  Serial.begin(9600);

  pinMode(statpin, OUTPUT);

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
  motor_pid_1.SetSampleTime(10);

  //---buttons
  pinMode(incr, INPUT);
  pinMode(decr, INPUT);
  digitalWrite(incr, INPUT_PULLUP);
  digitalWrite(decr, INPUT_PULLUP);
}

void loop()
{

//  if (Serial.available() > 0) {
//    // read the incoming byte:
//    incoming_data = Serial.parseInt();
//  }
//
//  pos_final = incoming_data; //set the desired position to the new position read in

  //---use buttons to increment or decrement the position
  if(digitalRead(incr) == LOW){
    pos_final += move_amount;
    delay(100);
  }
  else if(digitalRead(decr) == LOW){
    pos_final -= move_amount;
    delay(100);
  }

//  setpoint = act_pos * mult; //convert the position in mm to encoder counts
//  

  if (pos_final != pos_curr) { //if we are not where we need to be then we need to calculate a trajectory to get there

    //if it is a negative position the time needs its sign flipped:
    if ((pos_final - pos_curr) > 0) {
      time_final = 1000*(3 * sqrt((2 * (pos_final - pos_curr)) / acc)) / 2;
    }
    else {
      time_final = 1000*(3 * sqrt((-2 * (pos_final - pos_curr)) / acc)) / 2;
    }

    double vel = (3 * (pos_final - pos_curr)) / (2 * (time_final)); //calculate the velocity needed

    double time_b = (pos_curr - pos_final + vel * time_final) / vel; //calculate the time over the parabolic region

    Serial.print("Calculated time for trajectory: ");
    Serial.println(time_final);
    Serial.print("Calculated velocity for trajectory: ");
    Serial.println(vel);
    Serial.print("Calculated time for parabolas: ");
    Serial.println(time_b);

    //constants
    a[0] = pos_curr;
    a[1] = 0;
    a[2] = vel / (2 * time_b);

    b[0] = pos_final - (vel * pow(time_final, 2)) / (2 * time_b);
    b[1] = vel * time_final / time_b;
    b[2] = -vel / (2 * time_b);

    time_old = millis(); //make the old time reset
    
    while (i) {

      //get the current time in this loop
      time_new = millis();
      //Serial.print("time_new: ");
      //Serial.println(time_new);
      
      time_curr = time_new - time_old;
      //Serial.print("time_curr: ");
      //Serial.println(time_curr);
      
      //time_old = time_new;
      //Serial.print("time_old: ");
      //Serial.println(time_old);
      

      //get position if we are in the first parabolic region
      if (time_curr <= time_b) {
        setpoint = a[0] + a[1] * time_curr + a[2] * pow(time_curr, 2);
        Serial.println("In the 1st parabola");
      }
      //get position if we are in the linear region
      else if ((time_curr > time_b) && (time_curr <= (time_final - time_b))) {
        setpoint = ((pos_final + pos_curr - vel * time_final) / 2 + vel * time_curr);
        Serial.println("In the linear");
      }
      //get position if we are in the second parabolic region
      else if (time_curr > (time_final - time_b)) {
        setpoint = b[0] + b[1] * time_curr + b[2] * pow(time_curr, 2);
        Serial.println("In the 2nd parabola");
      }

      input = enc_1.read(); //get actual current position

      //move to new position
      if (input > setpoint) { //positive
        motor_pid_1.SetControllerDirection(REVERSE);
        motor_pid_1.Compute();
        set_motor_output(0, 2, output);
      }
      else { //negative
        motor_pid_1.SetControllerDirection(DIRECT);
        motor_pid_1.Compute();
        set_motor_output(0, 1, output);
      }

      //if we are at the end of the time we need to exit the loop
      if (time_curr >= time_final) {
        i = 0;
      }
    }
    pos_curr = pos_final; //update position
    i = 1;
  }

  //hold here until updated
  setpoint = pos_curr;

  input = enc_1.read(); //get actual current position

  //move to new position
  if (input > setpoint) { //positive
    motor_pid_1.SetControllerDirection(REVERSE);
    motor_pid_1.Compute();
    set_motor_output(0, 2, output);
  }
  else { //negative
    motor_pid_1.SetControllerDirection(DIRECT);
    motor_pid_1.Compute();
    set_motor_output(0, 1, output);
  }

//  Serial.print(input);
//  Serial.print(",");
//  Serial.print(setpoint);
//  Serial.print(",");
//  Serial.println(output);
//  Serial.println("Idle");
  delay(1);
}

void motorOff(int motor)
{
  // Initialize braked
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
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
