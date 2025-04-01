#!../../bin/linux-x86_64/rbd9103App

#- You may have to change default to something else
#- everywhere it appears in this file

< /epics/common/rocky-dev-netsetup.cmd

< envPaths

epicsEnvSet("ENGINEER", "J. Wlodek")
epicsEnvSet("SERIAL_PORT", "RBD_TTY")
epicsEnvSet("PORT", "RBD")
epicsEnvSet("P", "DEV:")
epicsEnvSet("R", "SB1:")
epicsEnvSet("PREFIX", "$(P)$(R)")

## Register all support components
dbLoadDatabase("../../dbd/rbd9103App.dbd",0,0)
rbd9103App_registerRecordDeviceDriver(pdbbase) 

drvAsynSerialPortConfigure("$(SERIAL_PORT)", "/dev/ttyUSB1", 0, 0, 0)

# For serial options, see the `Communications and Command Reference` section
# of the manual. This is the low-speed comms.
asynSetOption("$(SERIAL_PORT)", 0, "baud", 57600)
asynSetOption("$(SERIAL_PORT)", 0, "bits", 8)
asynSetOption("$(SERIAL_PORT)", 0, "parity", "none")
asynSetOption("$(SERIAL_PORT)", 0, "stop", 1)

asynOctetSetOutputEos("$(SERIAL_PORT)", 0, "\r\n")
asynOctetSetInputEos("$(SERIAL_PORT)", 0, "\r\n")

RBD9103Config("RBD", "$(SERIAL_PORT)")

# Load any additional specified databases.

## Load record instances
dbLoadTemplate("$(RBD_9103)/db/rbd9103.substitutions")

iocInit()

