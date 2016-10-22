# power meter
The following notes are about the Satec power meter BFM136/036 modbus queries to get the power data out for publishing to an EmonCMS system.

## pymodbus
The easiest way of doing this is to use pymodbus from what I can tell as we need to do modbus tcp on one side and post json to the webserver on the other side.

In Ubuntu do the following
```
$> pip install pymodbus
```
Then you could add this script to test whether the system works
```
#!/usr/bin/python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient

svr_addr = '192.168.1.203'

client = ModbusClient(svr_addr, port=502)
client.connect()

rr = client.read_holding_registers(46100, 6, unit=0x01)
assert(rr.registers[0] == 1805) # test the expected value
assert(rr.registers[1] == 6)    # test the expected value
assert(rr.registers[2] == 6101) # test the expected value
assert(rr.registers[3] == 8)    # test the expected value
assert(rr.registers[4] == 102)  # test the expected value
assert(rr.registers[5] == 2)    # test the expected value
```

## queries
These are the power queries that were performed on the last run of the system using the show everything window.

```
ti pi len ui fc reg    count
# DEVICE FIRMWARE
0  0  6   1  3  46100  6
# CURRENT PORT/IFACE
0  0  6   1  3  44342  2
# DEVICE MODEL ID
0  0  6   1  3  46080  14
# V1 - V3 INPUTS
0  0  6   1  3  46112  14

0  0  6   1  3  242    2
# WIRING MODE
0  0  6   1  3  46208  20
# DEVICE FIRMWARE
0  0  6   1  3  46100  6
# CURRENT PORT/IFACE
0  0  6   1  3  44342  2
# DEVICE MODEL ID
0  0  6   1  3  46080  14
# V1 - V3 INPUTS
0  0  6   1  3  46112  14

0  0  6   1  3  242    2
# WIRING MODE
0  0  6   1  3  46208  20

0  0  6   1  3  14720  18
0  0  6   1  3  14720  18
0  0  6   1  3  14720  18
0  0  6   1  3  14720  18
0  0  6   1  3  14720  18
0  0  6   1  3  46100  6
0  0  6   1  3  44342  2
0  0  6   1  3  46080  14
0  0  6   1  3  46112  14
0  0  6   1  3  242    2
0  0  6   1  3  46208  20
0  0  6   1  3  44342  1
0  0  6   1  3  46446  1
0  0  6   1  3  46928  48
0  0  6   1  3  46976  48
0  0  6   1  3  47024  48
0  0  6   1  3  46100  6
0  0  6   1  3  44342  2
0  0  6   1  3  46080  14
0  0  6   1  3  46112  14
0  0  6   1  3  242    2
0  0  6   1  3  46208  20
0  0  6   1  3  44342  1
0  0  6   1  3  46466  1
0  0  6   1  3  46928  48
0  0  6   1  3  46976  48
0  0  6   1  3  47024  48
0  0  6   1  3  14720  18
0  0  6   2  3  14720  18
0  0  6   3  3  14720  18
0  0  6   4  3  14720  18
0  0  6   5  3  14720  18
0  0  6   6  3  14720  18
0  0  6   7  3  14720  18
0  0  6   8  3  14720  18
0  0  6   9  3  14720  18
0  0  6   10 3  14720  18
.
.
.
.
.         26 3  14720  18
```

