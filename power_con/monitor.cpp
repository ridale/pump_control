#include "monitor.h"

Monitor::Monitor(int pin1, int pin2, int pin3) : 
            pinflow1(pin1),
            pinflow2(pin2),
            pinflow3(pin3),
            flow1(0),
            flow2(0),
            flow3(0),
            index(0)
{
  node[0].begin(9600);
  node[1].begin(9600);
  node[2].begin(9600);
}



/**
 * The main flow meter function reads the hall pulses.
 * Check this blog for information
 * http://forum.arduino.cc/index.php?topic=8548.0
 */
String Monitor::doFlow() {
  String retval("");
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
  retval = "{";
  retval += " \"type\":\"flow\"";
  sprintf(buff, "\"elapsed\":%i,", elapsed);
  retval += buff;

  sprintf(buff, "\"flow1\":%i,", tmp1);
  retval += buff;
  sprintf(buff, "\"flow2\":%i,", tmp2);
  retval += buff;
  sprintf(buff, "\"flow3\":%i,", tmp3);
  retval += buff;
  retval += "\"error\":\"NONE\"}";
  return retval;
}

/**
 * The main modbus function
 */
String Monitor::doModbus() {
  uint32_t addr;
  uint16_t registers[12];
  uint32_t reg_size = sizeof(registers)/sizeof(uint16_t);

  // set the modbus address to read
  addr = index%3;
  // read the data from modbus
  if (0 == readModbusRegisters(addr, registers, reg_size)) {
    return writeModbusXbee(addr, registers, reg_size);
  }
  else {
    return writeModbusXbeeErr(addr);
  }  
}
/**
 * Write the modbus data to the XBee 
 */
String Monitor::writeModbusXbee(int addr, uint16_t *data, uint32_t data_size) {
  String retval("");
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
String Monitor::writeModbusXbeeErr(int addr) {
  String retval("");
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
int Monitor::readModbusRegisters(int addr, uint16_t *data, uint32_t data_size)
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


