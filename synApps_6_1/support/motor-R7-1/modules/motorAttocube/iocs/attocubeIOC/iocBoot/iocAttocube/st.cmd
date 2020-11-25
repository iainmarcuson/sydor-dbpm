#!../../bin/linux-x86_64/attocube

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/attocube.dbd"
attocube_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=attocube:")

##
< ANC150.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("attocube:")

# Boot complete
