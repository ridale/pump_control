#!/usr/bin/python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient

svr_addr = '192.168.1.203'

client = ModbusClient(svr_addr, port=502)
client.connect()

rr = client.read_holding_registers(46100, 6, unit=0x01)
assert(rr.registers[0] == 1805)    # test the expected value
assert(rr.registers[1] == 6)       # test the expected value
assert(rr.registers[2] == 6101)    # test the expected value
assert(rr.registers[3] == 8)       # test the expected value
assert(rr.registers[4] == 102)     # test the expected value
assert(rr.registers[5] == 2)       # test the expected value

