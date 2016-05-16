/**
 * This is the power control code, this code controls 2 digital outputs
 * controlling opto-isolated relays switching 240V motor loads for
 * pump control on the farm.
 *
 * NOTE: We are keeping the state of the pins not checking them. We could
 *       use bitRead(PORTD, pin) or PORTB to get the digital pin states.
 *       Just seems over difficult...
 *
 * Commands:
 * {"command":"config"}
 * {"command":"status","pin":1}
 * {"command":"toggle","pin":1}
 * {"command":"on","pin":1}
 * {"command":"off","pin":1}
 *
 */
#include <ArduinoJson.h>

typedef struct myPin {
  int pin;
  uint32_t last_changed;
  bool state;
} pin_t;

/**
 * @brief The list of power pins in use
 *
 * The following is the default pin layout for
 * the DFRobotics relay board.
 */
pin_t pinPower[4] = { {2, 0, false},
                      {7, 0, false},
                      {8, 0, false},
                      {10, 0, false}
                    };

// string to hold inputCommand
String inputCommand = "";
// the command has been read
bool commandComplete = false;
// the delay between on/off in msecs
const int PIN_DEADBAND = 5000;
/**
 * Init function run before loop started
 * sets up direction of IO pins and serial
 * parameters
 */
void setup() {
  uint32_t now = millis();

  // Starting the other serial ports
  Serial1.begin(9600); // zigbee port

  // setup the digital outputs
  for (int i = 0; i < sizeof(pinPower)/sizeof(pin_t); i++ ) {
    pinMode(pinPower[i].pin, OUTPUT);
    digitalWrite(pinPower[i].pin,LOW);
    pinPower[i].last_changed = now;
  }
}
/**
 * The main run loop. I am using the serialEvent thingie
 * Means I should be running a read when data becomes
 * available, but it looks like voodoo to me.
 */
void loop() {
  // put your main code here, to run repeatedly:
  serialEvent1();
  if (commandComplete) {
    commandRespond(inputCommand);
    // clear the string:
    inputCommand = "";
    commandComplete = false;
  }
}
/**
 * Read the serial input waiting for a command
 */
void serialEvent1() {
  while (Serial1.available()) {
    char ch = (char) Serial1.read();
    inputCommand += ch;
    if (ch == '}') {
      commandComplete = true;
    }
  }
  // if we have no command incoming we can write out our status...
  if (inputCommand.length() == 0) {

  }
}
/**
 * Parse the JSON command packet and then do the
 * required action.
 * \param command the JSON string with the command object
 */
void commandRespond(String command) {
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(command);

  if (!root.success()) {
    Serial1.println("{");
    Serial1.println(" \"error\":\"Failed to parse JSON\"");
    Serial1.println("}");
    return;
  }
  const char* cmd = root["command"];
  if (NULL == cmd) {
    Serial1.println("{");
    Serial1.println(" \"error\":\"Failed to find command\"");
    Serial1.println("}");
  }
  else {
    String cmdVal = root["command"].asString();
    if (cmdVal == String("status")) {
      // command status
      returnStatus(root["pin"]);
    }
    else if(cmdVal == String("on")) {
      // command on
     turnOn(root["pin"]);
    }
    else if(cmdVal == String("off")) {
      // command off
       turnOff(root["pin"]);
    }
    else if(cmdVal == String("toggle")) {
      // command toggle
       togglePower(root["pin"]);
    }
    else if(cmdVal == String("config")) {
      // command config
      returnConfig();
    }
    else {
      // no idea what you want
      Serial1.println("{");
      cmdVal = " \"error\":\"Unknown command " + cmdVal + "\"";
      Serial1.println(" \"error\":\"Failed to parse command\"");
      Serial1.println("}");
    }
  }
}
/**
 * Writes the current status for the pin onto
 * the serial line
 * \param pinIndex the pin array index (0 based)
 */
void returnStatus(uint8_t pinIndex) {
  uint32_t pins = sizeof(pinPower)/sizeof(pin_t);
  char buff[128];

  Serial1.println("{");
  sprintf(buff, " \"pin\":%i,",pinIndex);
  Serial1.println(buff);

  if (pinIndex >= pins) {
    Serial1.println(" \"error\":\"PIN out of bounds\"");
  }
  else {
    sprintf(buff, " \"duration\":%i,", pinLastChanged(pinIndex));
    Serial1.println(buff);
    sprintf(buff, " \"on\":%s,", (pinPower[pinIndex].state)? "true":"false");
    Serial1.println(buff);
    Serial1.println(" \"error\":\"NONE\"");
  }
  Serial1.println("}");
}
/**
 * Give some configuration info back
 */
void returnConfig() {
  uint32_t pins = sizeof(pinPower)/sizeof(pin_t);
  char buff[128];

  Serial1.println("{");
  sprintf(buff, " \"pin_count\":%i,",pins);
  Serial1.println(buff);
  sprintf(buff, " \"deadband_ms\":%i", PIN_DEADBAND);
  Serial1.println(buff);
  Serial1.println("}");
}
/**
 * Get the interval in milliseconds since the output
 * Value of the pin last changed.
 */
uint32_t pinLastChanged(uint8_t pinIndex) {
  uint32_t now = millis();
  if (now > pinPower[pinIndex].last_changed) {
    return now - pinPower[pinIndex].last_changed;
  }
  else {
    return now + (UINT32_MAX - pinPower[pinIndex].last_changed);
  }

}
/**
 * Toggle the power state of the pin
 * \param pin the pin to flip
 * \returns 0 on success, -pin on failure
 */
void togglePower(uint8_t pinIndex) {
  uint32_t pins = sizeof(pinPower)/sizeof(pin_t);
  char buff[128];

  uint32_t last_change = pinLastChanged(pinIndex);
  if (last_change < PIN_DEADBAND) {
    Serial1.println("{");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    // too quick we have a deadband
    sprintf(buff, " \"error\":\"Deadband %ims, last change %ims\",",PIN_DEADBAND, last_change);
    Serial1.println(buff);
    Serial1.println("}");
  }
  else if (pinIndex >= pins) {
    Serial1.println("{");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    Serial1.println(" \"error\":\"PIN out of bounds\"");
    Serial1.println("}");
  }
  else {
    uint32_t now = millis();
    pinPower[pinIndex].last_changed = now;
    if (pinPower[pinIndex].state) {
      // turn off
      pinPower[pinIndex].state = false;
      digitalWrite(pinPower[pinIndex].pin,LOW);
      returnStatus(pinIndex);
    }
    else {
      // turn on
      pinPower[pinIndex].state = true;
      digitalWrite(pinPower[pinIndex].pin,HIGH);
      returnStatus(pinIndex);
    }
    pinPower[pinIndex].last_changed = now;
  }
}
/**
 * Turn a pin on, uses toggle to do the work
 * write error to serial port if index out
 * of range or already on
 */
void turnOn(uint8_t pinIndex) {
  uint32_t pins = sizeof(pinPower)/sizeof(pin_t);
  char buff[128];
  if (pinIndex >= pins) {
    togglePower(pinIndex); // let them write our error
  }
  else if (true == pinPower[pinIndex].state) {
    Serial1.println("{");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    Serial1.println("  \"error\":\"Already switched ON\"");
    Serial1.println("}");
  }
  else {
    togglePower(pinIndex);
  }
}
/**
 * Turn a pin off, uses toggle to do the work
 * write error to serial port if index out
 * of range or already off
 */
void turnOff(uint8_t pinIndex) {
  uint32_t pins = sizeof(pinPower)/sizeof(pin_t);
  char buff[128];
  if (pinIndex >= pins) {
    togglePower(pinIndex); // let them write our error
  }
  else if (false == pinPower[pinIndex].state) {
    Serial1.println("{");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    Serial1.println("  \"error\":\"Already switched OFF\"");
    Serial1.println("}");
  }
  else {
    togglePower(pinIndex);
  }
}

