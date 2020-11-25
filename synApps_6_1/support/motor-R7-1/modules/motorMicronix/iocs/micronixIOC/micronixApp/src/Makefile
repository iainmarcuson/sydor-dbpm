TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = micronix
# micronix.dbd will be created and installed
DBD += micronix.dbd

# micronix.dbd will be made up from these files:
micronix_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
micronix_DBD += asyn.dbd
micronix_DBD += drvAsynSerialPort.dbd
#endif
micronix_DBD += motorSupport.dbd
micronix_DBD += MicronixSupport.dbd

# Add all the support libraries needed by this IOC
micronix_LIBS += Micronix
micronix_LIBS += motor
#ifdef ASYN
micronix_LIBS += asyn
#endif
#ifdef SNCSEQ
micronix_LIBS += seq pv
#endif

# micronix_registerRecordDeviceDriver.cpp derives from micronix.dbd
micronix_SRCS += micronix_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
micronix_SRCS_DEFAULT += micronixMain.cpp
micronix_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#micronix_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
micronix_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

