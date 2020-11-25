#!../../bin/linux-x86_64/micronix

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/micronix.dbd"
micronix_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=micronix:")

## 
< motor.cmd.mmc200

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("micronix:")

# Boot complete
