# Ethernet
drvAsynIPPortConfigure("MAXNET","192.168.1.100:23",0,0,0)
# Serial
#!drvAsynSerialPortConfigure("MAXNET","/dev/ttyS0",0,0,0)
#!asynSetOption("MAXNET",0,"baud","115200")
#!asynSetOption("MAXNET",0,"bits","8")
#!asynSetOption("MAXNET",0,"parity","none")
#!asynSetOption("MAXNET",0,"crtscts","Y")

# The IEOS depends on the firmware version
#!asynOctetSetInputEos("MAXNET",0,"\n\r")
asynOctetSetInputEos("MAXNET",0,"\n")
asynOctetSetOutputEos("MAXNET",0,"\n")

dbLoadTemplate("MAXnet.substitutions")

# omsMAXnetConfig(
#    const char *portName,	/* MAXnet Motor Asyn Port name */
#    int numAxes,		/* Number of axes this controller supports */
#    const char *serialPortName,/* MAXnet Serial Asyn Port name */
#    int movingPollPeriod,	/* Time to poll (msec) when an axis is in motion */
#    int idlePollPeriod,	/* Time to poll (msec) when an axis is idle. 0 for no polling */
#    const char *initString)	/* Init String sent to card */
# Example init string:
#    "AX LH PSO; AY LH PSO; AZ LH PSO; AT LH PSO; AU LH PSO; AV LH PSO; AR LH PSO; AS LH PSO;"
omsMAXnetConfig("MAXnet1", 8, "MAXNET", 200, 2000, "AX LH PSO;")
