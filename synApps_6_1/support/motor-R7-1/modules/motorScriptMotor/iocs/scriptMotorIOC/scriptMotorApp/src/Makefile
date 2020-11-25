TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = scriptMotor
# scriptMotor.dbd will be created and installed
DBD += scriptMotor.dbd

# scriptMotor.dbd will be made up from these files:
scriptMotor_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
scriptMotor_DBD += asyn.dbd
scriptMotor_DBD += drvAsynIPPort.dbd
#endif
scriptMotor_DBD += luaSupport.dbd
scriptMotor_DBD += motorSupport.dbd
scriptMotor_DBD += ScriptMotorDriver.dbd

# Add all the support libraries needed by this IOC
scriptMotor_LIBS += ScriptMotor
scriptMotor_LIBS += motor
scriptMotor_LIBS += lua
#ifdef ASYN
scriptMotor_LIBS += asyn
#endif
#ifdef SNCSEQ
scriptMotor_LIBS += seq pv
#endif

# scriptMotor_registerRecordDeviceDriver.cpp derives from scriptMotor.dbd
scriptMotor_SRCS += scriptMotor_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
scriptMotor_SRCS_DEFAULT += scriptMotorMain.cpp
scriptMotor_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#scriptMotor_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
scriptMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

