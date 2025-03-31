TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application for testing


PROD_IOC = rbd9103App
DBD += $(PROD_IOC).dbd


# $(PROD_IOC).dbd will be made up from these files:
$(PROD_IOC)_DBD += base.dbd
$(PROD_IOC)_DBD += asyn.dbd asynRecord.dbd
$(PROD_IOC)_DBD += drvAsynSerialPort.dbd
$(PROD_IOC)_DBD += calc.dbd calcSupport.dbd
$(PROD_IOC)_DBD += iocAdmin.dbd busyRecord.dbd busySupport.dbd asSupport.dbd rbd9103.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_IOC)_SRCS_DEFAULT += $(PROD_IOC)_registerRecordDeviceDriver.cpp $(PROD_IOC)Main.cpp

$(PROD_IOC)_LIBS += asyn calc autosave devIocStats busy rbd9103
$(PROD_IOC)_LIBS += seq pv
$(PROD_IOC)_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
