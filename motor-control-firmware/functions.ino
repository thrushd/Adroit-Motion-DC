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
      Serial.println("SETPOSITION");
    }
    else if (part1.equalsIgnoreCase("getposition")) { //getposition command received
      received_data[0] = 2;
      received_data[1] = part2.toFloat(); //set the current axis
      received_data[2] = part3.toFloat(); //set the new position
      Serial.println("GETPOSITION");
    }
    else if (part1.equalsIgnoreCase("home")) { //homing command received
      received_data[0] = 3;
      received_data[1] = part2.toFloat(); //set the current axis
      received_data[2] = part3.toFloat(); //set the new position
      Serial.println("HOME");
    }
    else {
      Serial.println("ERROR"); //why are you sending me GARBAGE??!!
      received_data[0] = 0;
    }
  }
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
