
dbLoadTemplate("VME58.substitutions")

# OMS VME58 driver setup parameters: 
#     (1)cards, (2)base address(short, 4k boundary), 
#     (3)interrupt vector (0=disable or  64 - 255), (4)interrupt level (1 - 6),
#     (5)motor task polling rate (min=1Hz,max=60Hz)
oms58Setup(1, 0x1000, 190, 5, 10)
#!drvOms58debug = 4


