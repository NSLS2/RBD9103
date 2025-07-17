#!../../bin/linux-x86_64/rbd9103App

< /epics/common/xf31id1-ioc2-netsetup.cmd
< envPaths

epicsEnvSet("ENGINEER", "J. Wlodek")
epicsEnvSet("SERIAL_PORT", "RBD_TTY")
epicsEnvSet("PORT", "RBD")
epicsEnvSet("P", "DEV:")
epicsEnvSet("R", "RBD9103:")
epicsEnvSet("PREFIX", "$(P)$(R)")

## Register all support components
dbLoadDatabase("../../dbd/rbd9103App.dbd",0,0)
rbd9103App_registerRecordDeviceDriver(pdbbase)

drvAsynSerialPortConfigure("$(SERIAL_PORT)", "/dev/rbd9103_AQ01SPJH", 0, 0, 0)

# For serial options, see the `Communications and Command Reference` section
# of the manual. This is the low-speed comms.
asynSetOption("$(SERIAL_PORT)", 0, "baud", 57600)
asynSetOption("$(SERIAL_PORT)", 0, "bits", 8)
asynSetOption("$(SERIAL_PORT)", 0, "parity", "none")
asynSetOption("$(SERIAL_PORT)", 0, "stop", 1)

asynOctetSetOutputEos("$(SERIAL_PORT)", 0, "\r\n")
asynOctetSetInputEos("$(SERIAL_PORT)", 0, "\r\n")


RBD9103Config("RBD", "$(SERIAL_PORT)")
# Uncomment to enable debug logging.
#asynSetTraceMask("$(PORT)", 0, 0x9)

## Load record instances
dbLoadTemplate("$(RBD_9103)/db/rbd9103.substitutions")

iocInit()
