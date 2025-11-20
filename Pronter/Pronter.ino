char incoming_char;
char command_buffer[100]; // Buffer to store the incoming command
int buffer_index = 0;
bool command_complete = false;

void setup() {
  Serial.begin(9600); // Match this baud rate in Pronterface
  Serial.println("start"); // Send a startup message
}

void loop() {
  if (Serial.available() > 0) {
    incoming_char = Serial.read();

    if (incoming_char == '\n' || incoming_char == '\r') {
      // End of command detected
      command_buffer[buffer_index] = '\0'; // Null-terminate the string
      command_complete = true;
      buffer_index = 0; // Reset index for the next command
    } else {
      // Store the character in the buffer if there's room
      if (buffer_index < 99) {
        command_buffer[buffer_index++] = incoming_char;
      }
    }
  }

  if (command_complete) {
    // Process the command (e.g., call a G-code parser function)
    processGCode(command_buffer); 
    command_complete = false;

    // Send acknowledgement back to Pronterface
    Serial.println("ok"); 
  }

  // Other non-blocking tasks go here (e.g., temperature checks, motor steps)
}

void processGCode(char* command) {
  Serial.println(command);
}
