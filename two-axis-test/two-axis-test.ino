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
Encoder enc_1(14, 15); //encoder 1
Encoder enc_2(17, 18); //encoder 2
double mult_1 = 159.5; //counts per mm or deg for axis 1
double mult_2 = 157.48; //counts per mm or deg for axis 2

//---PID
//theta
double setpoint_1, input_1, output_1;
double k_p_1 = 1;
double k_i_1 = 0;
double k_d_1 = 0.008;
PID motor_pid_1(&input_1, &output_1, &setpoint_1, k_p_1, k_i_1, k_d_1, DIRECT);

//z
double setpoint_2, input_2, output_2;
double k_p_2 = 0.5;
double k_i_2 = 0;
double k_d_2 = 0.02;
PID motor_pid_2(&input_2, &output_2, &setpoint_2, k_p_2, k_i_2, k_d_2, DIRECT);

//---status
int statpin = 13;

//---buttons test
#define go 6

//---ejector coil
#define coil_pin 7

//---trajectory mapping
double acc_1 = 300000;             //acceleration to use
double acc_2 = 500000;             //acceleration to use
double pos_curr_1 = 0;          //current position
double pos_curr_2 = 0;          //current position
double pos_final_1 = 0;         //desired position
double pos_final_2 = 0;         //desired position
double time_final = 0;        //time to complete movement
double time_curr = 0;         //current time in the movement
double step_size = 1;         //step size time [ms]
double a[3];                  //constants for the polynomials
double b[3];
unsigned long time_new;       //used for timing
unsigned long time_old = 0;   //the last time used
long iterations = 0; //number of steps used in the movement
int i = 0;

//---serial testing
int incoming_data = 0;        // for incoming serial data

double axis_pos_1[2] = {90, 0}; //variables for the precoded movements
double axis_pos_2[2] = {80, 0};

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
  motor_pid_1.SetSampleTime(1); //KILOHERTZ BAYBEEEE

  motor_pid_2.SetMode(AUTOMATIC);
  motor_pid_2.SetSampleTime(1); //KILOHERTZ BAYBEEEE

  //---buttons
  pinMode(go, INPUT);
  pinMode(coil_pin, OUTPUT);
  digitalWrite(go, INPUT_PULLUP);
  digitalWrite(coil_pin, HIGH);
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
  if (digitalRead(go) == LOW) {
    //signal the move command

    Serial.println("Button pressed");

    //set_position_2(axis_pos_2[0]*mult_2); //move z axis
    set_position_1(axis_pos_1[0]*mult_1); //move theta axis
    delay(1000);
    shoot();
    delay(1000);
    set_position_1(axis_pos_1[1]*mult_1); //move theta axis
    //set_position_2(axis_pos_2[1]*mult_2); //move z axis

  }
  //hold here until updated
  else {
    setpoint_1 = pos_curr_1;
    setpoint_2 = pos_curr_2;

    input_1 = enc_1.read(); //get actual current position
    input_2 = enc_2.read(); //get actual current position

    //move to new position
    if (input_1 > setpoint_1) { //positive
      motor_pid_1.SetControllerDirection(REVERSE);
      motor_pid_1.Compute();
      set_motor_output(0, 2, output_1);
    }
    else { //negative
      motor_pid_1.SetControllerDirection(DIRECT);
      motor_pid_1.Compute();
      set_motor_output(0, 1, output_1);
    }

    //move to new position
    if (input_2 > setpoint_2) { //positive
      motor_pid_2.SetControllerDirection(REVERSE);
      motor_pid_2.Compute();
      set_motor_output(1, 2, output_2);
    }
    else { //negative
      motor_pid_2.SetControllerDirection(DIRECT);
      motor_pid_2.Compute();
      set_motor_output(1, 1, output_2);
    }

  }
  //plotting stuff for debugging PIDs, use serial plotter
  //    Serial.print(input);
  //    Serial.print(",");
  //    Serial.println(setpoint);
  //Serial.print(",");
  //Serial.println(output);
  //Serial.println("Idle");
  Serial.println(output_2);

}


void shoot() {

  digitalWrite(coil_pin, LOW);
  delay(300);
  digitalWrite(coil_pin, HIGH);

}

void set_position_1(double pos_final_1) {

  

  if (pos_final_1 != pos_curr_1) { //if we are not where we need to be then we need to calculate a trajectory to get there

    //if it is a negative position the time needs its sign flipped:
    if ((pos_final_1 - pos_curr_1) > 0) {
      time_final = 1000 * ( 3 * sqrt((2 * (pos_final_1 - pos_curr_1)) / acc_1)) / 2;
    }
    else {
      time_final = 1000 * ( 3 * sqrt((-2 * (pos_final_1 - pos_curr_1)) / acc_1)) / 2;
    }

    double vel = (3 * (pos_final_1 - pos_curr_1)) / (2 * (time_final)); //calculate the velocity needed

    double time_b = (pos_curr_1 - pos_final_1 + vel * time_final) / vel; //calculate the time over the parabolic region

    Serial.print("Calculated time for trajectory: ");
    Serial.println(time_final);
    Serial.print("Calculated velocity for trajectory: ");
    Serial.println(vel);
    Serial.print("Calculated time for parabolas: ");
    Serial.println(time_b);

    //constants
    a[0] = pos_curr_1;
    a[1] = 0;
    a[2] = vel / (2 * time_b);

    b[0] = pos_final_1 - (vel * pow(time_final, 2)) / (2 * time_b);
    b[1] = vel * time_final / time_b;
    b[2] = -vel / (2 * time_b);

    //    //make an array with all of the positions needed for this move
    iterations = time_final / step_size; //gets the number of iterations for a move based on the time and step size, eg 10 seconds would result in 10000 iterations with a step size of 1ms

    Serial.print("Itetrations = ");
    Serial.println(iterations);

    double set_array[iterations];

    for ( int i = 0; i < iterations; i++) {
      //set position for the 1st parabola
      if (i <= time_b) {
        set_array[i] = a[0] + a[1] * i + a[2] * pow(i, 2);
        //Serial.println("In the 1st parabola");
      }
      //set position if we are in the linear region
      else if ((i > time_b) && (i <= (time_final - time_b))) {
        set_array[i] = ((pos_final_1 + pos_curr_1 - vel * time_final) / 2 + vel * i);
        //Serial.println("In the linear");
      }
      //set position if we are in the second parabolic region
      else if (i > (time_final - time_b)) {
        set_array[i] = b[0] + b[1] * i + b[2] * pow(i, 2);
        //Serial.println("In the 2nd parabola");
      }
    }

    int j = 0;

    while (j < iterations) {

      time_curr = millis();

      if (time_curr - time_old > step_size) {

        time_old = time_curr;

        //get position if we are in the first parabolic region
        if (time_curr <= time_b) {
          setpoint_1 = set_array[j];
          //Serial.println("In the 1st parabola");
        }
        //get position if we are in the linear region
        else if ((time_curr > time_b) && (time_curr <= (time_final - time_b))) {
          setpoint_1 = set_array[j];
          //Serial.println("In the linear");
        }
        //get position if we are in the second parabolic region
        else if (time_curr > (time_final - time_b)) {
          setpoint_1 = set_array[j];
          //Serial.println("In the 2nd parabola");
        }
        j += 1; //update j;
      }

      input_1 = enc_1.read(); //get actual current position

      //move to new position
      if (input_1 > setpoint_1) { //positive
        motor_pid_1.SetControllerDirection(REVERSE);
        motor_pid_1.Compute();
        set_motor_output(0, 2, output_1);
      }
      else { //negative
        motor_pid_1.SetControllerDirection(DIRECT);
        motor_pid_1.Compute();
        set_motor_output(0, 1, output_1);
      }
    }
    pos_curr_1 = pos_final_1; //update position
  }

}

void set_position_2(double pos_final_2) {

  if (pos_final_2 != pos_curr_2) { //if we are not where we need to be then we need to calculate a trajectory to get there

    //if it is a negative position the time needs its sign flipped:
    if ((pos_final_2 - pos_curr_2) > 0) {
      time_final = 1000 * ( 3 * sqrt((2 * (pos_final_2 - pos_curr_2)) / acc_2)) / 2;
    }
    else {
      time_final = 1000 * ( 3 * sqrt((-2 * (pos_final_2 - pos_curr_2)) / acc_2)) / 2;
    }

    double vel = (3 * (pos_final_2 - pos_curr_2)) / (2 * (time_final)); //calculate the velocity needed

    double time_b = (pos_curr_2 - pos_final_2 + vel * time_final) / vel; //calculate the time over the parabolic region

    Serial.print("Calculated time for trajectory: ");
    Serial.println(time_final);
    Serial.print("Calculated velocity for trajectory: ");
    Serial.println(vel);
    Serial.print("Calculated time for parabolas: ");
    Serial.println(time_b);

    //constants
    a[0] = pos_curr_2;
    a[1] = 0;
    a[2] = vel / (2 * time_b);

    b[0] = pos_final_2 - (vel * pow(time_final, 2)) / (2 * time_b);
    b[1] = vel * time_final / time_b;
    b[2] = -vel / (2 * time_b);

    //    //make an array with all of the positions needed for this move
    iterations = time_final / step_size; //gets the number of iterations for a move based on the time and step size, eg 10 seconds would result in 10000 iterations with a step size of 1ms

    Serial.print("Itetrations = ");
    Serial.println(iterations);

    double set_array[iterations];

    for ( int i = 0; i < iterations; i++) {
      //set position for the 1st parabola
      if (i <= time_b) {
        set_array[i] = a[0] + a[1] * i + a[2] * pow(i, 2);
        //Serial.println("In the 1st parabola");
      }
      //set position if we are in the linear region
      else if ((i > time_b) && (i <= (time_final - time_b))) {
        set_array[i] = ((pos_final_2 + pos_curr_2 - vel * time_final) / 2 + vel * i);
        //Serial.println("In the linear");
      }
      //set position if we are in the second parabolic region
      else if (i > (time_final - time_b)) {
        set_array[i] = b[0] + b[1] * i + b[2] * pow(i, 2);
        //Serial.println("In the 2nd parabola");
      }
    }

    int j = 0;

    while (j < iterations) {

      time_curr = millis();

      if (time_curr - time_old > step_size) {

        time_old = time_curr;

        //get position if we are in the first parabolic region
        if (time_curr <= time_b) {
          setpoint_2 = set_array[j];
          //Serial.println("In the 1st parabola");
        }
        //get position if we are in the linear region
        else if ((time_curr > time_b) && (time_curr <= (time_final - time_b))) {
          setpoint_2 = set_array[j];
          //Serial.println("In the linear");
        }
        //get position if we are in the second parabolic region
        else if (time_curr > (time_final - time_b)) {
          setpoint_2 = set_array[j];
          //Serial.println("In the 2nd parabola");
        }
        j += 1; //update j;
      }

      input_2 = enc_2.read(); //get actual current position

      //move to new position
      if (input_2 > setpoint_2) { //positive
        motor_pid_2.SetControllerDirection(REVERSE);
        motor_pid_2.Compute();
        set_motor_output(1, 2, output_2);
      }
      else { //negative
        motor_pid_2.SetControllerDirection(DIRECT);
        motor_pid_2.Compute();
        set_motor_output(1, 1, output_2);
      }
    }
    pos_curr_2 = pos_final_2; //update position
  }

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
