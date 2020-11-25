#!../../bin/linux-x86_64/scriptMotor

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/scriptMotor.dbd"
scriptMotor_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=scriptMotor:")

## 

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("scriptMotor:")

# Boot complete
