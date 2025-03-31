// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>

// EPICS includes
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <iocsh.h>

#include <asynOctetSyncIO.h>

#include "rbd9103.h"

// Error message formatters
#define ERR(msg)                                                                                 \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "ERROR | %s::%s: %s\n", driverName, functionName, \
              msg)

#define ERR_ARGS(fmt, ...)                                                              \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "ERROR | %s::%s: " fmt "\n", driverName, \
              functionName, __VA_ARGS__);

// Warning message formatters
#define WARN(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WARN | %s::%s: %s\n", driverName, functionName, msg)

#define WARN_ARGS(fmt, ...)                                                            \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "WARN | %s::%s: " fmt "\n", driverName, \
              functionName, __VA_ARGS__);

// Log message formatters
#define LOG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", driverName, functionName, msg)

#define LOG_ARGS(fmt, ...)                                                                       \
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);


/**
 * @brief External configuration function for RBD9103.
 *
 * Envokes the constructor to create a new RBD9103 object
 * This is the function that initializes the driver, and is called in the IOC startup script
 *
 */
extern "C" int RBD9103Config(const char* portName, const char* serialPortName) {
    new RBD9103(portName, serialPortName);
    return (asynSuccess);
}

static void exitCallbackC(void* pPvt) {
    RBD9103* pRBD9103 = (RBD9103*)pPvt;
    delete pRBD9103;
}

static void samplingThreadC(void* pPvt){
    RBD9103* pRBD9103 = (RBD9103*)pPvt;
    pRBD9103->samplingThread();
}


asynStatus RBD9103::writeReadCmd(const char* cmd, char* ret, size_t maxChars, double timeout){

    size_t nwrite, nread;
    asynStatus status;
    int eomReason;

    const char *functionName="writeReadCmd";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    status = pasynOctetSyncIO->writeRead(this->pasynUserSerialPort, cmd,
                                      strlen(cmd), ret, maxChars, timeout,
                                      &nwrite, &nread, &eomReason);

    return status;
}


void RBD9103::sampleOnce(){
    char ret[64];
    status = writeReadCmd("&S", ret, sizeof(ret), TIMEOUT);
    if (status != asynSuccess) {
        ERR("Failed to read sampling response!");
        return;
    }

    // Response echos back &S and then follows with SamplingRate=$RATE,
    // &S=,Range=002nA,+0.0008,nA
    if(ret[2] == '='){
        setIntegerParam(this->RBD9103_Stable, 1);
    } else {
        setIntegerParam(this->RBD9103_Stable, 0);
    }

    char* sampleToken=strtok(ret, ",");
    char* range=strtok(NULL, ",");
    char* current=strtok(NULL, ",");
    char* unit=strtok(NULL, ",");

    // Get the units
    if (strcmp(units, "nA") == 0) {
        setIntegerParam(this->RBD9103_Units, RBD_UNT_NA);
    } else if (strcmp(units, "uA") == 0) {
        setIntegerParam(this->RBD9103_Units, RBD_UNT_UA * 1000);
    } else if (strcmp(units, "mA") == 0) {
        setIntegerParam(this->RBD9103_Units, RBD_UNT_MA * 1000000);
    }

    if(current[0] == '+'){
        current++;
    }
    setDoubleParam(this->RBD9103_Current, atof(current));

    callParamCallbacks();
}


void RBD9103::samplingThread(){
    const char* functionName = "samplingThread";
    double samplingRate;

    getDoubleParam(this->RBD9103_SamplingRate, &samplingRate);

    while(this->sampling){
        this->sampleOnce();
        epicsThreadSleep(sampling);
    }

    setIntegerParam(RBD9103_Sample, 0);
    setIntegerParam(RBD9103_Stable, 0);
    callParamCallbacks();
}


asynStatus RBD9103::writeInt32(asynUser* pasynUser, epicsInt32 value){
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char* functionName = "writeInt32";
    if (function == RDB9103_Sample){
        if(value == 0 && !this->sampling){
            ERR("Sampling is already stopped!");
            return asynError;
        } else if(value == 1 && this->sampling){
            ERR("Sampling is already started!");
            return asynError;
        } else if(value == 1 && !this->sampling){
            this->sampling = true;
            this->samplingThread = epicsThreadCreate("RBD9103_Sampling",
                                                    epicsThreadPriorityMedium,
                                                    epicsThreadGetStackSize(epicsThreadStackMedium),
                                                    (EPICSTHREADFUNC) samplingThreadC,
                                                    (void*) this);
        } else if(value == 0 && this->sampling){
            this->sampling = false;
            epicsThreadMustJoin(this->samplingThread);
        }
    } else if (function == RBD9103_SampleOnce){
        if (this->sampling){
            ERR("Sampling is already started!");
            return asynError;
        } else {
            this->sampleOnce();
        }
    } else if (function == RDB9103_Bias) {
        char cmd[32], ret[64];
        snprintf(cmd, sizeof(cmd), "&B%d", value);
        status = writeReadCmd(cmd, ret, sizeof(ret), TIMEOUT);
        if (status != asynSuccess) {
            ERR("Failed to set bias setting!");
            return status;
        }
    }

    if (status) {
        ERR_ARGS("ERROR status=%d, function=%d, value=%f, msg=%s", status, function, value);
    } else {
        setIntegerParam(function, value);
        LOG_ARGS("function=%d value=%f", function, value);
    }

    callParamCallbacks();
    return status;
}


RBD9103::RBD9103(const char* portName, const char* serialPortName)
    : asynPortDriver(
          portName, 1, /* maxAddr */
          (int)NUM_RBD9103_PARAMS,
          asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask |
              asynOctetMask, /* Interface mask */
          asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask |
              asynOctetMask, /* Interrupt mask */
          0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
          1, /* Autoconnect */
          0, /* Default priority */
          0) /* Default stack size*/

{
    const char* functionName = "RBD9103";

    pasynOctetSyncIO->connect(serialPortName, 0, &this->pasynUserSerialPort, NULL);

    createParam(RBD9103_ModelString, asynParamOctet, &this->RBD9103_Model);
    createParam(RBD9103_DrvVersionString, asynParamOctet, &this->RBD9103_DrvVersion);
    createParam(RBD9103_NumDigitsString, asynParamInt32, &this->RBD9103_NumDigits);

    setStringParam(this->RBD9103_DrvVersion, RBD9103_DRIVER_VERSION);
    char ret[32];
    asynStatus status = writeReadCmd("&K", ret, sizeof(ret), TIMEOUT);
    if (status != asynSuccess) {
        ERR("Failed to get model number");
        return;
    }

    // Response echos back &K and then follows with Key=$MODEL,
    // so split on the equal sign to get the model number alone.
    char* model = strtok(ret, "=");
    strtok(NULL, "="); // Skip the first token
    setStringParam(this->RBD9103_Model, model);

    callParamCallbacks();
    LOG_ARGS("Connected to ammeter: %s", model);

    epicsAtExit(exitCallbackC, (void*) this);
}

RBD9103::~RBD9103(){
    const char* functionName = "~RBD9103";

    LOG("Shutting down RBD 9103 IOC...");
    if(this->samplingThread != NULL && this->sampling){
        this->sampling = false;
        epicsThreadMustJoin(this->samplingThread);
    }

    LOG("Done");
}



//-------------------------------------------------------------
// RBD9103 ioc shell registration
//-------------------------------------------------------------

/* RBD9103Config -> These are the args passed to the constructor in the epics config function */
static const iocshArg RBD9103ConfigArg0 = {"portName", iocshArgString};

// This parameter must be customized by the driver author. Generally a URL, Serial Number, ID, IP
// are used to connect.
static const iocshArg RBD9103ConfigArg1 = {"serialPortNameName", iocshArgString};

// This parameter must be customized by the driver author. Generally a URL, Serial Number, ID, IP
// are used to connect.
static const iocshArg RBD9103ConfigArg2 = {"numChannels", iocshArgInt};

/* Array of config args */
static const iocshArg* const RBD9103ConfigArgs[] = {&RBD9103ConfigArg0, &RBD9103ConfigArg1};

/* what function to call at config */
static void configRBD9103CallFunc(const iocshArgBuf* args) {
    RBD9103Config(args[0].sval, args[1].sval);
}

/* information about the configuration function */
static const iocshFuncDef configRBD9103 = {"RBD9103Config", 2, RBD9103ConfigArgs};

/* IOC register function */
static void RBD9103Register(void) { iocshRegister(&configRBD9103, configRBD9103CallFunc); }

/* external function for IOC register */
extern "C" {
epicsExportRegistrar(RBD9103Register);
}
