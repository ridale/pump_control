/**
 * This module uses RS485 and modbus to read the power data
 * from a SPM91 DIN rail single phase energy meter from
 * zhuhai pilot technology co. ltd. The following library is
 * used for modbus 
 * http://playground.arduino.cc/Code/ModbusMaster
 * 
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
#include <Arduino.h>
#include <ModbusMaster.h>

class Monitor {
  String writeModbusXbee(int addr, uint16_t *data, uint32_t data_size);
  String writeModbusXbeeErr(int addr);
  int readModbusRegisters(int addr, uint16_t *data, uint32_t data_size);
  ModbusMaster node[3] = {ModbusMaster(1), ModbusMaster(2), ModbusMaster(3)};

public:
  Monitor(int pin1, int pin2, int pin3);
  volatile int flow1;
  volatile int flow2;
  volatile int flow3;
  const int pinflow1;
  const int pinflow2;
  const int pinflow3;
  uint32_t index;

  String doModbus();
  String doFlow();
};

