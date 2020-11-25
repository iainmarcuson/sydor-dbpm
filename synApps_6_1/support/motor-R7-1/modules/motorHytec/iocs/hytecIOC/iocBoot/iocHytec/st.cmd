## Example vxWorks startup file

## The following is needed if your board support package doesn't at boot time
## automatically cd to the directory containing its startup script
#cd "/home/username/epics/iocs/hytecIOC/iocBoot/iocHytec"

< cdCommands
#< ../nfsCommands

cd topbin

## You may have to change hytec to something else
## everywhere it appears in this file
ld 0,0, "hytec.munch"

cd top

## Register all support components
dbLoadDatabase "dbd/hytec.dbd"
hytec_registerRecordDeviceDriver pdbbase

cd startup

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=hytec:")

## IPAC carrier should be configured here

## An example needs to be created based on README_Hy8601Asyn
#!< Hytec8601.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("hytec:")

# Boot complete
