#include <PID_v1.h>
#include <Encoder.h>
#include <Metro.h>

//---motors
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
long old_position = -999;
Encoder enc_0(14, 15); //encoder 0
Encoder enc_1(17, 18); //encoder 1

//---PID
double setpoint_pos_0, input_pos_0, output_pos_0; //axis 0 positional
double setpoint_pos_1, input_pos_1, output_pos_1; //axis 0 positional
double setpoint_vel_0, input_vel_0, output_vel_0, signed_setpoint_vel_0; //axis 0 velocity
double setpoint_vel_1, input_vel_1, output_vel_1, signed_setpoint_vel_1; //axis 1 velocity

PID motor_pos_0(&setpoint_pos_0, &input_pos_0, &output_pos_0, 1, 0, 0.001, DIRECT); //position PID for axis 0
PID motor_pos_1(&setpoint_pos_1, &input_pos_1, &output_pos_1, 1, 0, 0.001, DIRECT); //position PID for axis 1
PID motor_vel_0(&setpoint_vel_0, &input_vel_0, &output_vel_0, 1, 0, 0.001, DIRECT); //velocity PID for axis 0
PID motor_vel_1(&setpoint_vel_1, &input_vel_1, &output_vel_1, 1, 0, 0.001, DIRECT); //velocity PID for axis 1

float received_data[3]; //data received from the host

//---velocity control
float velocity = 0;
long count = 0;
long oldposition = 0;
long newposition;
int interval = 2;
int ppr[2] = {2000, 2000};

Metro velocity_metro(interval);

int traj_flag = 0;
int position_count = 0;
float current_position[2] = {0, 0}; //array to hold the positions of the axis
long acceleration[2] = {500000, 1000000}; //accelerations for different axis
float mult[2] = {159.5, 157.48}; //translation of counts to mm / deg for axis
int time_step = 1;
float *position_array;
int iterations = 0;

void setup() {
  //---debug
  Serial.begin(9600);

  // Initialize digital pins as outputs for motors
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

  //set PWM frequency
  analogWriteFrequency(pwmpin[0], 187500);
  analogWriteFrequency(pwmpin[1], 187500);

  //---PID
  motor_pos_0.SetMode(AUTOMATIC);
  motor_pos_0.SetSampleTime(1);
  motor_pos_1.SetMode(AUTOMATIC);
  motor_pos_1.SetSampleTime(1);
  motor_vel_0.SetMode(AUTOMATIC);
  motor_vel_0.SetSampleTime(1);
  motor_vel_1.SetMode(AUTOMATIC);
  motor_vel_1.SetSampleTime(1);

}

void loop() {

  get_command(received_data); //update based on commands received

  //set new variables based on the new data so I don't have to use stupid indexes

  if (received_data[0] == 1) { //set position
    //is a current trajectory calculated for position?
    if (traj_flag == 0) {
      iterations = calc_trajectory(current_position[(int)received_data[1]], received_data[2], time_step, acceleration[(int)received_data[1]], position_array);
      Serial.println(iterations);
      traj_flag = 1;
    }
    else {
      if (position_count != iterations) {
        set_position(position_array[position_count], (int)received_data[1]); //move an increment in the array
        Serial.println(position_array[position_count]);
        position_count++; //increment positional count
      }
      else {
        received_data[0] = 4; //reset to holding
        traj_flag = 0; //reset trajectory flag
        position_count = 0; //reset positional count
        Serial.println("Movement complete");
      }
    }
  }
  else if (received_data[0] == 2) { //get position
    //send current position
    //reset to holding
  }
  else if (received_data[0] == 3) { //home
    //home
    //reset to hold
  }
  else if (received_data[0] == 0) { //no valid command
    //Serial.println("The fuck is this?");
    //delay(1);
  }
  else { //hold
    //Serial.println("holding");
    //delay(1);
  }
}




