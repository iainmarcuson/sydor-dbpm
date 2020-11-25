## Example vxWorks startup file

## The following is needed if your board support package doesn't at boot time
## automatically cd to the directory containing its startup script
#cd "/home/username/epics/iocs/omsAsynIOC/iocBoot/iocOmsAsyn"

< cdCommands
#< ../nfsCommands

cd topbin

## You may have to change omsAsyn to something else
## everywhere it appears in this file
ld 0,0, "omsAsyn.munch"

## Register all support components
cd top
dbLoadDatabase "dbd/omsAsyn.dbd"
omsAsyn_registerRecordDeviceDriver pdbbase
cd startup

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=omsAsyn:")

## 
< MAXnet.cmd
< MAXv.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("omsAsyn:")

# Boot complete
