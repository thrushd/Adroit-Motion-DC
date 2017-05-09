String command;

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {

  get_command();
  
  if(position_updated){
    //calculate new trajectory
    
  }

  //every set time interval update the setposition

  //update motor position

}

void get_command(void) {

  while (Serial.available()) //as long as there is data in the buffer
  {
    digitalWrite(13, HIGH); //blink the status LED

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

    if (part1.equalsIgnoreCase("setposition")) { //setposition command received
      int axis = part2.toInt(); //set the current axis
      float new_position = part3.toFloat(); //set the new position
      //update the desired position
      Serial.println("SETPOSITION");
    }
    else if (part1.equalsIgnoreCase("getposition")) { //getposition command received
      int axis = part2.toInt();
      //send back current position for selected axis
      //Serial.println(current_position);
      Serial.println("GETPOSITION");
    }
    else if (part1.equalsIgnoreCase("home")) { //homing command received
      int axis = part2.toInt(); //set current axis
      //do homing command
      Serial.println("HOMING");
    }
    else {
      Serial.println("ERROR COMMAND NOT RECOGNIZED"); //why are you sending me GARBAGE??!!
    }
    digitalWrite(13, LOW); //turn off status LED
    command = "";
  }
}


