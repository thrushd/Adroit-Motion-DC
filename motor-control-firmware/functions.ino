//---Get Commands---//
/*
Input: array of size 3 that will be modified to return the command, axis, and position
Returns: nothing

Gets commands being sent over the serial port and parses them. If they match the set commands
it modifies the array passed to it with those values. If it is unrecognized it returns a 0.

Index | Name             | Value
0     | Command type     | 0-3
1     | Selected axis    | 0-1
2     | Desired position | Anything

Command Name     | Value
No valid command | 0
Set position     | 1
Get postion      | 2
Home             | 3
Hold             | 4

An example command sent to the controller:
setposition, 1, 143.32
*/

void get_command(float received_data[]) {

	String command; //create string to hold command

	while (Serial.available()) { //as long as there is data in the buffer

		char c = Serial.read(); //store one character
		command += c; //append the character to the string
	}

	if (command.length() != 0) { //if we have a command we need to parse and update things

		String part1; //string will be split into three parts
		String part2;
		String part3;

		int first_index = command.indexOf(","); //need to set the indexes in order to parse properly
		int second_index = command.indexOf(",", first_index + 1);

		part1 = command.substring(0, first_index); //get command string
		part2 = command.substring(first_index + 1, command.indexOf(",", first_index + 1)); //get axis
		part3 = command.substring(second_index + 1, command.indexOf(",", second_index + 1)); //get position (if available, if not results in command string

		//depending on what command we got set different values
		if (part1.equalsIgnoreCase("setposition")) { //setposition command received
			received_data[0] = 1;
			received_data[1] = part2.toFloat(); //set the current axis
			received_data[2] = part3.toFloat(); //set the new position
			//Serial.println("SETPOSITION");
		}
		else if (part1.equalsIgnoreCase("getposition")) { //getposition command received
			received_data[0] = 2;
			received_data[1] = part2.toFloat(); //set the current axis
			received_data[2] = part3.toFloat(); //set the new position
			//Serial.println("GETPOSITION");
		}
		else if (part1.equalsIgnoreCase("home")) { //homing command received
			received_data[0] = 3;
			received_data[1] = part2.toFloat(); //set the current axis
			received_data[2] = part3.toFloat(); //set the new position
			//Serial.println("HOME");
		}
		else {
			Serial.println("ERROR"); //why are you sending me GARBAGE??!!
			received_data[0] = 0;
		}
	}
}

//---Calculate Movement Trajectory---//
int calc_trajectory(float current_position, float desired_position, int time_step, float acceleration, float **position_array, int axis) {

	// Serial.print("Desired Position: ");
	// Serial.println(desired_position);
	// Serial.print("Current Position: ");
	// Serial.println(current_position);

	if (current_position != desired_position) { //if we are not where we need to be then we need to calculate a trajectory to get there

		float time_final = 0;
		//if it is a negative position the time needs its sign flipped:
		if ((desired_position - current_position) > 0) {
			time_final = 1000 * ( 3 * sqrt((2 * (desired_position - current_position)) / acceleration)) / 2;
		}
		else {
			time_final = 1000 * ( 3 * sqrt((-2 * (desired_position - current_position)) / acceleration)) / 2;
		}

		float velocity = (3 * (desired_position - current_position)) / (2 * (time_final)); //calculate the velocity needed

		float time_b = (current_position - desired_position + velocity * time_final) / velocity; //calculate the time over the parabolic region

		//    debugging
		// Serial.print("Calculated time for trajectory: ");
		// Serial.println(time_final);
		// Serial.print("Calculated velocity for trajectory: ");
		// Serial.println(velocity);
		// Serial.print("Calculated time for parabolas: ");
		// Serial.println(time_b);

		//constants
		float a[3], b[3];

		a[0] = current_position;
		a[1] = 0;
		a[2] = velocity / (2 * time_b);

		b[0] = desired_position - (velocity * pow(time_final, 2)) / (2 * time_b);
		b[1] = velocity * time_final / time_b;
		b[2] = -velocity / (2 * time_b);

		//make an array with all of the positions needed for this move
		int iterations = time_final / time_step; //gets the number of iterations for a move based on the time and step size, eg 10 seconds would result in 10000 iterations with a step size of 1ms

		//debugging
		// Serial.print("Number of itetrations: ");
		// Serial.println(iterations);

		*position_array = (float*) malloc(sizeof(float) * iterations);

		if (position_array != NULL) {

			for ( int i = 0; i < iterations; i++) {
				//set position for the 1st parabola
				if (i <= time_b) {
					(*position_array)[i] = a[0] + a[1] * i + a[2] * pow(i, 2);
					// Serial.print("First Parabola: ");
					// Serial.println((*position_array)[i]);
				}
				//set position if we are in the linear region
				else if ((i > time_b) && (i <= (time_final - time_b))) {
					(*position_array)[i] = ((desired_position + current_position - velocity * time_final) / 2 + velocity * i);
					// Serial.print("Linear ");
					// Serial.println((*position_array)[i]);
				}
				//set position if we are in the second parabolic region
				else if (i > (time_final - time_b)) {
					(*position_array)[i] = b[0] + b[1] * i + b[2] * pow(i, 2);
					// Serial.print("Second Parabola ");
					// Serial.println((*position_array)[i]);
				}
			}

			//play it back for debugging
			// for (int i = 0; i < iterations; i++) {
				// Serial.println((*position_array)[i]);
			// }

			return iterations;
		}
		else { //oh shit thats no good
			Serial.print("Position array is null");
			return -1;
		}

	}
	return -1;
	Serial.println("No movement to make");
}

//---Update selected axis---//
void update_axis(int axis) {

	if (axis_status[axis] == 1) { //set position
		
		//is a current trajectory calculated for position?
		if (traj_flag[axis] == 0) {
			
			//update the correct axis
			if(axis == 0){
				iterations[axis] = calc_trajectory(current_position[axis], target_position[axis], time_step, acceleration[axis], &position_array_0, axis);
			}
			else if(axis == 1){
				iterations[axis] = calc_trajectory(current_position[axis], target_position[axis], time_step, acceleration[axis], &position_array_1, axis);
			}

			//play it back for debugging
			//      for (int i = 0; i < iterations[0]; i++) {
			//        Serial.println(position_array[i]);
			//      }

			traj_flag[axis] = 1;
		}
		//if not we need to move
		else {
			if (position_count[axis] != iterations[axis]) {

				unsigned long current_time = millis();

				if (current_time - prev_time[axis] > interval_test) {

					prev_time[axis] = current_time;

					if(axis == 0){
						set_position(position_array_0[position_count[axis]], axis); //move an increment in the array
					}
					else if(axis == 1){
						set_position(position_array_1[position_count[axis]], axis); //move an increment in the array
					}
					
					//Serial.println(position_array[position_count[axis]]);
					
					position_count[axis]++; //increment positional count
				}
				else {
					//Serial.println("Waiting");
				}
			}
			else {// done with movement
				axis_status[axis] = 4; //reset to holding
				traj_flag[axis] = 0; //reset trajectory flag
				position_count[axis] = 0; //reset positional count
				current_position[axis] = target_position[axis]; //update the current position
				
//				Serial.print("Movement complete for axis: ");
//				Serial.println(axis);
        Serial.println("OK");
        
        
				if(axis == 0){
					free(position_array_0);
				}
				else if(axis == 1){
					free(position_array_1);
				}
				
			}
		}
	}
	else if (axis_status[axis] == 2) { //get position
		
		if(axis == 0){
			Serial.println(enc_0.read()/mult[axis]);
		}
		else if(axis == 1){
			Serial.println(enc_1.read()/mult[axis]);
		}
		
		axis_status[axis] = 4; //reset to holding
	}
	else if (axis_status[axis] == 3) { //home
		home_axis(axis);
    Serial.println("OK");
		axis_status[axis] = 4; //reset to holding
	}
	else if (axis_status[axis] == 0) { //no valid command
		//Serial.println("The fuck is this?");
		//delay(1);
		axis_status[axis] = 4;
	}
	else { //hold
		//Serial.println("holding");
		//delay(1);
		set_position(current_position[axis], axis);
	}
}

//---Update Position---//
void set_position(float setpoint, int axis) {

	if (axis == 0) {

		input_pos_0 = enc_0.read(); //get actual current position

		setpoint_pos_0 = (double)setpoint;

		//move to new position
		if (input_pos_0 > setpoint_pos_0) { //positive
			motor_pos_0.SetControllerDirection(REVERSE);
			motor_pos_0.Compute();
			set_motor_output(axis, 2, output_pos_0);
		}
		else { //negative
			motor_pos_0.SetControllerDirection(DIRECT);
			motor_pos_0.Compute();
			set_motor_output(axis, 1, output_pos_0);

		}

	}
	else if (axis == 1) {

		input_pos_1 = enc_1.read(); //get actual current position

		setpoint_pos_1 = setpoint;

		//move to new position
		if (input_pos_1 > setpoint_pos_1) { //positive
			motor_pos_1.SetControllerDirection(REVERSE);
			motor_pos_1.Compute();
			set_motor_output(axis, 2, output_pos_1);
		}
		else { //negative
			motor_pos_1.SetControllerDirection(DIRECT);
			motor_pos_1.Compute();
			set_motor_output(axis, 1, output_pos_1);
		}
	}
	else {
		Serial.println("No valid axis given for position update");
	}
}

//---Set Velocity---//
void set_velocity(float signed_setpoint, int axis) {

 if (axis == 0) {

    unsigned long current_time = millis();

    if (current_time - prev_time[axis] > interval_test) {

      prev_time[axis] = current_time;

      newposition = enc_0.read(); //find the new position
      count = newposition - oldposition; //find the count since the last interval
      velocity = ((count / ppr[0]) * 60) / (interval / 1000); //calculate velocity
      oldposition = newposition; //set the old position
    }


    //---do PID stuff
    if (signed_setpoint > 0) { //positive
      input_vel_0 = velocity;
      setpoint_vel_0 = signed_setpoint;
      motor_vel_0.Compute();
      set_motor_output(axis, 2, output_vel_0);
    }
    else { //negative
      input_vel_0 = abs(velocity);
      setpoint_vel_0 = abs(signed_setpoint);
      motor_vel_0.Compute();
      set_motor_output(axis, 1, output_vel_0);
    }
  }
  else if (axis == 1) {

    unsigned long current_time = millis();

    if (current_time - prev_time[axis] > interval_test) {

      prev_time[axis] = current_time;

      newposition = enc_1.read(); //find the new position
      count = newposition - oldposition; //find the count since the last interval
      velocity = ((count / ppr[1]) * 60) / (interval / 1000); //calculate velocity
      oldposition = newposition; //set the old position
    }

    //---do PID stuff
    if (signed_setpoint > 0) { //positive
      input_vel_1 = velocity;
      setpoint_vel_1 = signed_setpoint;
      motor_vel_1.Compute();
      set_motor_output(axis, 1, output_vel_1);
    }
    else { //negative
      input_vel_1 = abs(velocity);
      setpoint_vel_1 = abs(signed_setpoint);
      motor_vel_1.Compute();
      set_motor_output(axis, 2, output_vel_1);
      //Serial.println("Negative");
    }
  }
  else {
    Serial.println("No valid axis given for velocity move");
  }
}

//---Homing---//

void home_axis(int axis) {

  //Serial.println("Beginning home routine");

  while (digitalRead(axis_limits[axis]) == HIGH) { //quickly advance to limit switch
    set_velocity(1, axis);
  }

  set_velocity(0, axis); //stop moving

  motor_off(axis); //no really, stop moving

  current_position[axis] = 0; //reset current position

  if (axis == 0) {
    enc_0.write(0);
  }
  else {
    enc_1.write(0);
  }

  //Serial.println("Home complete");
}

//---Set Motor Output---//
/*
Inputs: motor number, direction, pwm value
Returns: nothing

Will set a motor going in a specific direction the motor will continue
going in that direction, at that speed until told to do otherwise.

direct: Should be between 0 and 3, with the following result
0: Brake to VCC
1: Clockwise
2: CounterClockwise
3: Brake to GND

pwm: should be a value between 0 and 255, higher the number, the faster
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

//---Turn motors off---//
void motor_off(int axis)
{
	// Initialize braked
	for (int i = 0; i < 2; i++)
	{
		digitalWrite(inApin[i], LOW);
		digitalWrite(inBpin[i], LOW);
	}
	analogWrite(pwmpin[axis], 0);
}
