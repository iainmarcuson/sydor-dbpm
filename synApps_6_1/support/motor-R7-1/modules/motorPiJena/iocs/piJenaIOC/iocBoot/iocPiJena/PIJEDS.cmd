drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)

dbLoadTemplate("PIJEDS.substitutions")

# Piezosystem Jena EDS motor controller/driver setup parameters:
#     (1) maximum number of controllers in system
#     (2) maximum drives per controller
#     (3) motor task polling rate (min=1Hz, max=60Hz)
#drvPIJEDSdebug=1
PIJEDSSetup(1, 2, 60)

# Piezosystem Jena EDS controller/driver configuration parameters:
#     (1) controller being configured
#     (2) asyn port name (string)
#     (3) asyn address (GPIB)
PIJEDSConfig(0, "serial1", 0)
