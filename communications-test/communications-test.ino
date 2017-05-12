float data[3];

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {

  get_command(data);

  for (int i = 0; i < (sizeof(data)/sizeof(float)); i++) {
    Serial.print(data[i]);
    Serial.print("   ");
  }

  Serial.println();
  delay(1);

}

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
      Serial.println("SET POSITION");
    }
    else if (part1.equalsIgnoreCase("getposition")) { //getposition command received
      received_data[0] = 2;
      received_data[1] = part2.toFloat(); //set the current axis
      received_data[2] = part3.toFloat(); //set the new position
      Serial.println("GET POSITION");
    }
    else if (part1.equalsIgnoreCase("home")) { //homing command received
      received_data[0] = 3;
      received_data[1] = part2.toFloat(); //set the current axis
      received_data[2] = part3.toFloat(); //set the new position
      Serial.println("HOME");
    }
    else {
      Serial.println("ERROR COMMAND NOT RECOGNIZED"); //why are you sending me GARBAGE??!!
      received_data[0] = 0;
    }
    command = ""; //reset the command. Prolly not neccisary, but I don't know what's going on
  }
}


