
# The MVP2001 needs an RS232-to-RS485 adapter
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)
#!asynOctetSetOutputEos("serial1", 0, "\r")
#!asynOctetSetInputEos("serial1", 0, "\r\n")

dbLoadTemplate("MVP2001.substitutions")

# MicroMo MVP2001 driver setup parameters: 
#
# NOTE: The 1st controller on each chain should have it's address = 1.
#       The rest of the controllers on a chain should follow sequentially.
#
# MVP2001CreateController(
#        port name,
#        serial port,
#        number of axes,
#        Moving poll period (ms),
#        Idle poll period (ms))
MVP2001CreateController("MVP1", "serial1", 1, 250, 1000)

# MVP2001CreateAxis(
#        controller port,
#        axis index,
#        encoder lines per rev,
#        max current (mA)),
#        limit polarity (1=NO,0=NC)
MVP2001CreateAxis("MVP1", 0, 16, 1000, 0)
