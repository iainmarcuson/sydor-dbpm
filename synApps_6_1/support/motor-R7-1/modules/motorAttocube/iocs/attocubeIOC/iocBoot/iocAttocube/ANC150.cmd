
drvAsynSerialPortConfigure("serial1","/dev/ttyS0",0,0,0)
#!asynSetOption("serial", -1, "baud", "38400")
#!asynOctetSetInputEos("serial1",0,"")
#!asynOctetSetOutputEos("serial1",0,"")

dbLoadTemplate("ANC150.substitutions")

# attocube ANC 150 asyn motor driver setup parameter.
ANC150AsynSetup(1)  /* number of ANC150 controllers in system.  */

# attocube ANC 150 asyn motor driver configure parameters.
#     (1) Controller number being configured
#     (2) ASYN port name
#     (3) Number of axes this controller supports
#     (4) Time to poll (msec) when an axis is in motion
#     (5) Time to poll (msec) when an axis is idle. 0 for no polling
ANC150AsynConfig(0, "serial1", 3, 250, 2000)
