#!../../bin/linux-x86_64/mclennan

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/mclennan.dbd"
mclennan_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=mclennan:")

## 
< PM304.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("mclennan:")

# Boot complete
