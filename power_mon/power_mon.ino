/**
 * This module uses RS485 and modbus to read the power data
 * from a SPM91 DIN rail single phase energy meter from
 * zhuhai pilot technology co. ltd.
 * The final part of the puzzle will be to write that data to 
 * xbee module attached to the DFR0219 (dfrobot IO expander)
 * 
 * There is also some ISRs attached to some of the digital IO 
 * pins to read rising edges of the hall effect sensor output
 * from the water flow meters attached to them.
 * 
 * NOTE: The other serial ports (1,2,3) are flakey at 115200
 *       I saw a lot of corrupt writes, you could up from the
 *       9600, too hard reconfiguring the 2 XBees to test
 * 
 * NOTE: The DUE has a 32 bit int, the smaller boards use 16 bit
 *       this may be important for the flow accumulators.
 */
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

/**
 * Init function run before loop started
 * sets up direction of IO pins and serial
 * & modbus parameters
 */
void setup()
{
  // initialize modbus comms
  node[0].begin(9600);
  node[1].begin(9600);
  node[2].begin(9600);
  // Starting the other serial ports
  Serial1.begin(9600); // zigbee port
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
 * Main entry point loop, run after setup
 */
void loop()
{
  static uint32_t i;
  // loop counter
  i++;
  doModbus(i);
  //Serial1.flush();
  doFlow();
  //Serial1.flush();

  // poll every half a second (1.5 second updates per channel, there is some serial delay ~200ms)
  // the modbus timeout delay sits at about 2200ms
  delay(300);
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
  Serial1.println("  \"type\":\"flow\"");
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
  Serial1.println("  \"type\":\"power\"");
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
  Serial1.println("  \"type\":\"power\"");
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

