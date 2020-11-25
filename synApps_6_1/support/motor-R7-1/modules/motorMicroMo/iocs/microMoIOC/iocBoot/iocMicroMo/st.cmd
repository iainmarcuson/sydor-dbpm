#!../../bin/linux-x86_64/microMo

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/microMo.dbd"
microMo_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=microMo:")

## 
< MVP2001.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("microMo:")

# Boot complete
