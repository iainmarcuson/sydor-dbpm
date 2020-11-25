
drvAsynSerialPortConfigure("serial1", "/dev/ttyS0", 0, 0, 0)

dbLoadTemplate("PM304.substitutions")

# Mclennan PM304 driver setup parameters:
#     (1) maximum number of controllers in system
#     (2) motor task polling rate (min=1Hz, max=60Hz)
PM304Setup(1, 10)

# Mclennan PM304 driver configuration parameters:
#     (1) controller being configured
#     (2) MPF serial server name (string)
#     (3) Number of axes on this controller
PM304Config(0, "serial1", 1)
