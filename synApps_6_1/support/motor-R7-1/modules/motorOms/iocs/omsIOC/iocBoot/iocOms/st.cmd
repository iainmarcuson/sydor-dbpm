## Example vxWorks startup file

## The following is needed if your board support package doesn't at boot time
## automatically cd to the directory containing its startup script
#cd "/home/USERNAME/epics/ioc/omsIOC/iocBoot/iocOms"

< cdCommands
#< ../nfsCommands

cd topbin

## You may have to change oms to something else
## everywhere it appears in this file
ld 0,0, "oms.munch"

## Register all support components
cd top
dbLoadDatabase "dbd/oms.dbd"
oms_registerRecordDeviceDriver pdbbase

cd startup

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=oms:")

## Support for the OMS MAXv (VxWorks shell or iocsh)
#< MAXv.cmd
#< MAXv.iocsh

## Support for the OMS VME85
#< VME58.cmd

## Support for the OMS VME8 and VME44
#< VME8.cmd

## Support for the OMS PC68 and PC78 (requires asyn)
#< PC68.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("oms:")

# Boot complete
