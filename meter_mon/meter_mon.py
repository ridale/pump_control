#!/usr/bin/python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
from pymodbus.payload import BinaryPayloadDecoder as PayloadDecoder

svr_addr = '192.168.1.203'

client = ModbusClient(svr_addr, port=502)
client.connect()

result = client.read_holding_registers(46100, 6, unit=0x01)
print "Firmware V(%d.%d) RF(%d.%d) BootLoader(%d.%d)" %(
    rr.registers[0], rr.registers[1], rr.registers[2],
    rr.registers[3], rr.registers[4], rr.registers[5])

result = client.read_holding_registers(13312, 30, unit=0x01)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Little)
decoded = {
    'V1 Voltage': decoder.decode_32bit_uint(),
    'V2 Voltage': decoder.decode_32bit_uint(),
    'V3 Voltage': decoder.decode_32bit_uint(),
    'I1 Voltage': decoder.decode_32bit_uint(),
    'I2 Voltage': decoder.decode_32bit_uint(),
    'I3 Voltage': decoder.decode_32bit_uint(),
    'kW L1': decoder.decode_32bit_int(),
    'kW L2': decoder.decode_32bit_int(),
    'kW L3': decoder.decode_32bit_int(),
    'kvar L1': decoder.decode_32bit_int(),
    'kvar L2': decoder.decode_32bit_int(),
    'kvar L3': decoder.decode_32bit_int(),
    'kVA L1': decoder.decode_32bit_uint(),
    'kVA L2': decoder.decode_32bit_uint(),
    'kVA L3': decoder.decode_32bit_uint(),
    'Power factor L1': decoder.decode_32bit_int(),
    'Power factor L2': decoder.decode_32bit_int(),
    'Power factor L3': decoder.decode_32bit_int(),
}

for name, value in decoded.iteritems():
    print"%s = %d\n" %(name, value)

result = client.read_holding_registers(14720, 4, unit=0x01)
decoder = BinaryPayloadDecoder.fromRegisters(result.registers, endian=Endian.Little)
decoded = {
    'KWh import': decoder.decode_32bit_uint(),
    'KWh export': decoder.decode_32bit_uint(),
}

for name, value in decoded.iteritems():
    print"%s = %d\n" %(name, value)

