/*
 Will set a motor going in a specific direction the motor will continue 
 going in that direction, at that speed until told to do otherwise.
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 255, higher the number, the faster
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

//---calculate velocity

//velocity = velocity(motor to use (encoder), interval to sample over (ms), pulses per rev of the encoder)
////variables
//long newposition;
//long oldposition = 0;
//unsigned long newtime;
//unsigned long oldtime = 0;
//double count = 0;
//double velocity;
//int interval = 50;
//int ppr = 2000;
//
//double velocity(Encoder &the_enc){
// 
//
//
//  Serial.print("Velocity: ");
//  Serial.println(velocity);
//  return velocity;
//}
  
  
//---do PID stuff

//  if(signed_setpoint > 0){ //positive
//    input = velocity;
//    setpoint = signed_setpoint;
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, output);
//    //Serial.println("Positive");
//  }
//  else { //negative
//    input = abs(velocity);
//    setpoint = abs(signed_setpoint);
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, output);
//    //Serial.println("Negative");
//  }

//if(signed_setpoint > 0){ //positive
//    
//    input = velocity(enc_1, 50, 2000);
//    
//    setpoint = signed_setpoint;
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, output);
//    //Serial.println("Positive");
//  }
//  else { //negative
//    
//    input = abs(velocity(enc_1, 50, 2000));
//    
//    setpoint = abs(signed_setpoint);
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, output);
//    //Serial.println("Negative");
//  }

//---postion control
//  input = enc_1.read();
//
//  if (input > setpoint) { //positive
//    motor_pid_1.SetControllerDirection(REVERSE);
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 2, output);
//  }
//  else { //negative
//    motor_pid_1.SetControllerDirection(DIRECT);
//    motor_pid_1.Compute();
//    set_motor_output(in_a_1, in_b_1, pwm_pin_1, 1, output);
//  }

