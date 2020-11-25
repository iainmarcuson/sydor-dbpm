#!../../bin/linux-x86_64/mxmotor

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/mxmotor.dbd"
mxmotor_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=mxmotor:")

##


iocInit

## motorUtil (allstop & alldone)
motorUtilInit("mxmotor:")

# Boot complete
