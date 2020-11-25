TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = mxmotor
# mxmotor.dbd will be created and installed
DBD += mxmotor.dbd

# mxmotor.dbd will be made up from these files:
mxmotor_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
mxmotor_DBD += asyn.dbd
#endif
mxmotor_DBD += motorSupport.dbd
ifdef MX
mxmotor_DBD += devMXmotor.dbd

# Add all the support libraries needed by this IOC
mxmotor_LIBS += MXmotor
endif
mxmotor_LIBS += motor
#ifdef ASYN
mxmotor_LIBS += asyn
#endif
#ifdef SNCSEQ
mxmotor_LIBS += seq pv
#endif

# mxmotor_registerRecordDeviceDriver.cpp derives from mxmotor.dbd
mxmotor_SRCS += mxmotor_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
mxmotor_SRCS_DEFAULT += mxmotorMain.cpp
mxmotor_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#mxmotor_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
mxmotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

