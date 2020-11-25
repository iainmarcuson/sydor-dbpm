
dbLoadTemplate("VME8.substitutions")

# OMS VME driver setup parameters: 
#     (1)cards, (2)base address(short, 16-byte boundary), 
#     (3)interrupt vector (0=disable or  64 - 255), (4)interrupt level (1 - 6),
#     (5)motor task polling rate (min=1Hz,max=60Hz)
omsSetup(1, 0xFC00, 180, 5, 10)
#!drvOMSdebug = 4

