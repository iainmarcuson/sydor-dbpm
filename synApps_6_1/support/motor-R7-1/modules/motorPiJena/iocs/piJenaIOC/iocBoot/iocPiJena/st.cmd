#!../../bin/linux-x86_64/piJena

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/piJena.dbd"
piJena_registerRecordDeviceDriver pdbbase

cd "${TOP}/iocBoot/${IOC}"

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=piJena:")

##
< PIJEDS.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("piJena:")

# Boot complete
