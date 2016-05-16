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
#include <ModbusMaster.h>

// create 3 modbus objects for the power meters
ModbusMaster node[3] = {ModbusMaster(1), ModbusMaster(2), ModbusMaster(3)};

// Variables for the flow meter
volatile int flow1;
const int pinflow1 = 2;
volatile int flow2;
const int pinflow2 = 3;
volatile int flow3;
const int pinflow3 = 4;

/**
 * ISR for flow meter 2
 */
void accum1() {
  flow1++;
}
/**
 * ISR for flow meter 2
 */
void accum2() {
  flow2++;
}
/**
 * ISR for flow meter 3
 */
void accum3() {
  flow3++;
}

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

const int flow[3] =  {11,12,13};

// string to hold inputCommand
String inputCommand = "";
// the command has been read
bool commandComplete = false;
// the delay between on/off in msecs
const int PIN_DEADBAND = 5000;

unsigned long now = 0;

/**
 * Init function run before loop started
 * sets up direction of IO pins and serial
 * parameters
 */
void setup() {
  now = millis();
  // initialize modbus comms
  node[0].begin(9600);
  node[1].begin(9600);
  node[2].begin(9600);
  // Starting the other serial ports
  Serial1.begin(9600); // zigbee port
  //Serial.begin(9600);
  // setup the digital outputs
  for (int i = 0; i < sizeof(pinPower)/sizeof(pin_t); i++ ) {
    pinMode(pinPower[i].pin, OUTPUT);
    digitalWrite(pinPower[i].pin,LOW);
    pinPower[i].last_changed = now;
  }
  //Serial2.begin(9600);
  //Serial3.begin(9600);
  // setup the flow rate pins
  pinMode(pinflow1, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinflow1), accum1, RISING);
  pinMode(pinflow2, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinflow2), accum2, RISING);
  pinMode(pinflow3, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinflow3), accum3, RISING);
}
/**
 * The main run loop. I am using the serialEvent thingie
 * Means I should be running a read when data becomes
 * available, but it looks like voodoo to me.
 */
void loop() {
  // check got data on the serial line
  serialEvent1();
  // if we have the json close } then process
  if (commandComplete) {
    commandRespond(inputCommand);
    // clear the string:
    inputCommand = "";
    commandComplete = false;
  }
}

bool notyet(unsigned long theNow) {
  return ((theNow - now) > 300);
}

/**
 * Read the serial input waiting for a command
 */
void serialEvent1() {
  static uint32_t i;
  unsigned long  theNow = millis();
  String modbusString("");

  while (Serial1.available()) {
    char ch = (char) Serial1.read();
    inputCommand += ch;
    if (ch == '}') {
      commandComplete = true;
    }
  }
  // if we have no command incoming we can write out our status...
  if (inputCommand.length() == 0 && notyet(theNow)) {
    doModbus(i);
    doFlow();
    i++;
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
    Serial1.println("{\"type\":\"command\",");
    Serial1.println(" \"error\":\"Failed to parse JSON\"");
    Serial1.println("}");
    return;
  }
  const char* cmd = root["command"];
  if (NULL == cmd) {
    Serial1.println("{\"type\":\"command\",");
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
      Serial1.println("{\"type\":\"command\",");
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

  Serial1.println("{\"type\":\"command\",");
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

  Serial1.println("{\"type\":\"command\",");
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
    Serial1.println("{\"type\":\"command\",");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    // too quick we have a deadband
    sprintf(buff, " \"error\":\"Deadband %ims, last change %ims\",",PIN_DEADBAND, last_change);
    Serial1.println(buff);
    Serial1.println("}");
  }
  else if (pinIndex >= pins) {
    Serial1.println("{\"type\":\"command\",");
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
    Serial1.println("{\"type\":\"command\",");
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
    Serial1.println("{\"type\":\"command\",");
    sprintf(buff, " \"pin\":%i,",pinIndex);
    Serial1.println(buff);
    Serial1.println("  \"error\":\"Already switched OFF\"");
    Serial1.println("}");
  }
  else {
    togglePower(pinIndex);
  }
}

/**
 * The main flow meter function reads the hall pulses.
 * Check this blog for information
 * http://forum.arduino.cc/index.php?topic=8548.0
 */
void doFlow() {
  char buff[128];
  static uint32_t last_time;
  uint32_t elapsed = 0;
  int32_t tmp1,tmp2,tmp3;
  // get interval
  uint32_t now = millis();
  if (now > last_time) {
    elapsed = now - last_time;
  }
  else {
    // overflow (cant find maxint #def)
    elapsed = now + (UINT32_MAX - last_time);
  }
  last_time = now;
  // disable interrupts
  noInterrupts();
  // get values & reset counters
  tmp1 = flow1;
  flow1 = 0;
  tmp2 = flow2;
  flow2 = 0;
  tmp3 = flow3;
  flow3 = 0;
  // enable interrupts
  interrupts();
  // write values
  Serial1.println("{");
  Serial1.println("  \"type\":\"flow\",");
  sprintf(buff, "  \"elapsed\":%i,", elapsed);
  Serial1.println(buff);

  sprintf(buff, "  \"flow1\":%i,", tmp1);
  Serial1.println(buff);
  sprintf(buff, "  \"flow2\":%i,", tmp2);
  Serial1.println(buff);
  sprintf(buff, "  \"flow3\":%i,", tmp3);
  Serial1.println(buff);

  Serial1.println("  \"error\":\"NONE\"");
  Serial1.println("}");
}

/**
 * The main modbus function
 */
void doModbus(int counter) {
  uint32_t addr;
  uint16_t registers[12];
  uint32_t reg_size = sizeof(registers)/sizeof(uint16_t);

  // set the modbus address to read
  addr = counter%3;
  // read the data from modbus
  if (0 == readModbusRegisters(addr, registers, reg_size)) {
    writeModbusXbee(addr, registers, reg_size);
  }
  else {
    writeModbusXbeeErr(addr);
  }
}

/**
 * Write the modbus data to the XBee
 */
void writeModbusXbee(int addr, uint16_t *data, uint32_t data_size) {
  uint8_t j;
  char buff[256];
  Serial1.println("{");
  Serial1.println("  \"type\":\"power\",");
  sprintf(buff, "  \"addr\":%i,", addr+1);
  Serial1.println(buff);
  for (j = 0; j < data_size; j++) {
      sprintf(buff, "  \"%i\":%i,",40000+j+1, data[j]);
      Serial1.println(buff);
  }
  Serial1.println("  \"error\":\"NONE\"");
  Serial1.println("}");
}
/**
 * Write the failure to the XBee
 */
void writeModbusXbeeErr(int addr) {
  char buff[64];
  Serial1.println("{");
  Serial1.println("  \"type\":\"power\",");
  sprintf(buff, "  \"addr\":%i,", addr+1);
  Serial1.println(buff);
  Serial1.println("  \"error\":\"COMMS ERROR\"");
  Serial1.println("}");
}

/**
 * Reads the modbus registers and fills the data array
 */
int readModbusRegisters(int addr, uint16_t *data, uint32_t data_size)
{
  uint8_t j, result;
  // slave: read (12) 16-bit holding registers (0x03) starting at register 0 to RX buffer
  result = node[addr].readHoldingRegisters(0, 12);
  // do something with data if read is successful
  if (result == node[addr].ku8MBSuccess) {
    for (j = 0; j < data_size; j++)
    {
      data[j] = node[addr].getResponseBuffer(j);
    }
  }
  else {
    return -1;
  }
  return 0;
}
