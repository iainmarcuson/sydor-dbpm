
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)

dbLoadTemplate("C300.substitutions")

# C300CreateController(
#      port name,
#      serial port,
#      num axes,
#      moving poll period (ms),
#      idle poll period (ms))
C300CreateController("C300", "serial1", 3, 250, 2000)
