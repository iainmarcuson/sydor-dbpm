TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = oms
# oms.dbd will be created and installed
DBD += oms.dbd

# oms.dbd will be made up from these files:
oms_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
oms_DBD += asyn.dbd
#endif
oms_DBD += motorSupport.dbd
oms_DBD += devOms.dbd

# Add all the support libraries needed by this IOC
oms_LIBS += oms
oms_LIBS += motor
#ifdef ASYN
oms_LIBS += asyn
#endif
#ifdef SNCSEQ
oms_LIBS += seq pv
#endif

# oms_registerRecordDeviceDriver.cpp derives from oms.dbd
oms_SRCS += oms_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
oms_SRCS_DEFAULT += omsMain.cpp
oms_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#oms_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
oms_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

