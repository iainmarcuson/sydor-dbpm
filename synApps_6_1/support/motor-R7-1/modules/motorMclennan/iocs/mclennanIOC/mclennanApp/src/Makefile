TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = mclennan
# mclennan.dbd will be created and installed
DBD += mclennan.dbd

# mclennan.dbd will be made up from these files:
mclennan_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
mclennan_DBD += asyn.dbd
mclennan_DBD += drvAsynSerialPort.dbd
#endif
mclennan_DBD += motorSupport.dbd
mclennan_DBD += devMclennanMotor.dbd

# Add all the support libraries needed by this IOC
mclennan_LIBS += Mclennan
mclennan_LIBS += motor
#ifdef ASYN
mclennan_LIBS += asyn
#endif
#ifdef SNCSEQ
mclennan_LIBS += seq pv
#endif

# mclennan_registerRecordDeviceDriver.cpp derives from mclennan.dbd
mclennan_SRCS += mclennan_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
mclennan_SRCS_DEFAULT += mclennanMain.cpp
mclennan_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#mclennan_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
mclennan_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

