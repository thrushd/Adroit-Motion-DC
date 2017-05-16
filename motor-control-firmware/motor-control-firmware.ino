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

PID motor_pos_0(&input_pos_0, &output_pos_0, &setpoint_pos_0, 1, 0, 0.008, DIRECT); //position PID for axis 0 //theta
PID motor_pos_1(&input_pos_1, &output_pos_1, &setpoint_pos_1, 0.5, 0, 0.002, DIRECT); //position PID for axis 1 //z
PID motor_vel_0(&input_vel_0, &output_vel_0, &setpoint_vel_0, 1, 0, 0.001, DIRECT); //velocity PID for axis 0
PID motor_vel_1(&input_vel_1, &output_vel_1, &setpoint_vel_1, 1, 0, 0.001, DIRECT); //velocity PID for axis 1

float received_data[3]; //data received from the host

//---velocity control
float velocity = 0;
long count = 0;
long oldposition = 0;
long newposition;
int interval = 2;
int ppr[2] = {2000, 2000};

Metro velocity_metro = Metro(interval);

int time_step = 1; //ms
Metro motor_movement = Metro(time_step);

int traj_flag[2]; //whether or not the trajectory needs to be calculated
int axis_status[2]; //flag used to hold the current status of each axis
float setposition[2]; //the position set by the master
int iterations[2]; //returned number of iterations needed for movement
int position_count[2]; //used to keep track of where we are in the movement
float current_position[2] = {0, 0}; //array to hold the positions of the axis
long acceleration[2] = {300000, 500000}; //accelerations for different axis
float mult[2] = {159.5, 157.48}; //translation of counts to mm / deg for axis

float *position_array;

//-------------------------------------------------------------------------------------------
long prev_time = 0;
long interval_test = time_step;



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

  if (Serial.available()) {

    get_command(received_data); //update if there is data

    if (received_data[1] == 0) { //axis 0
      axis_status[0] = received_data[0]; //update the command
      setposition[0] = received_data[2]*mult[0]; //update position
    }
    else if (received_data[1] == 1) { //axis 1
      axis_status[1] = received_data[0]; //update the command
      setposition[1] = received_data[2]*mult[1]; //update position
    }
    else {
      Serial.println("Cannot update status flags");
    }
  }

  //only axis 0 will be updating for now
  update_axis(0); //update axis 0
  update_axis(1); //update axis 1
  
}




