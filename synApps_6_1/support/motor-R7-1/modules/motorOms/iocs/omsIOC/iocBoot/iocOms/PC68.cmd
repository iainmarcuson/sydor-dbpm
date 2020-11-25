
dbLoadTemplate("PC68.substitutions")

# OMS PC68/78 stand-alone serial driver setup parameters: 
#     (1) maximum number of controllers in system
#     (2) motor task polling rate (min=1Hz,max=60Hz)
OmsPC68Setup(1, 10)

# OMS PC68/78 stand-alone serial driver configuration parameters: 
#     (1) Card# being configured
#     (2) asyn port name
OmsPC68Config(0, "L0")
#!drvOmsPC68debug=4
