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

using namespace std;

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


string RBD9103::writeReadCmd(const char* cmd){

    size_t nwrite, nread;
    asynStatus status;
    int eomReason;
    char ret[64];

    const char *functionName="writeReadCmd";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    status = pasynOctetSyncIO->writeRead(this->pasynUserSerialPort, cmd,
                                      strlen(cmd), ret, 64, TIMEOUT,
                                      &nwrite, &nread, &eomReason);

    if (status != asynSuccess) {
        ERR("Failed to write command!");
        return string("");
    }
    return string(ret);
}

// Firmware Version: 02.09
// Build: 1-25-18
// R, Range=AutoR
// I, sample Interval=0500 mSec
// L, Chart Log Update Interval=0200 mSec
// B, BIAS=OFF
// F, Filter=032
// V, FormatLen=5
// CA, Autocal=OFF
// G, AutoGrounding=DISABLED
// Q, State=MEASURE
// P, PID=NEW_DEVICE


string RBD9103::splitRespOnDelim(string resp, const char* delim){
    return resp.substr(resp.find(delim) + strlen(delim), resp.length());
}



asynStatus RBD9103::getDeviceStatus(){
    const char* functionName = "getDeviceStatus";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    size_t nwrite, nread;
    int eomReason;
    asynStatus status = pasynOctetSyncIO->write(this->pasynUserSerialPort, "&Q", 2, 1, &nwrite);

    printf("Reading device status:\n");
    bool readDeviceID = false;
    char ret[64];
    while(!readDeviceID){
        status = pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret), 1, &nread, &eomReason);
        string statusLine = string(ret);
        printf("%s\n", statusLine.c_str());
        if(status != asynSuccess){
            ERR("Failed to read device status!");
            return status;
        } else if(statusLine.rfind("Firmware", 0) == 0) {
            const char* fwVersion = splitRespOnDelim(statusLine, ": ").c_str();
            setStringParam(this->RBD9103_FwVersion, fwVersion);
        } else if(statusLine.rfind("Build", 0) == 0) {
            const char* buildDate = splitRespOnDelim(statusLine, ": ").c_str();
            setStringParam(this->RBD9103_BuildDate, buildDate);
        } else if(statusLine.rfind("R, Range", 0) == 0) {
            string range = splitRespOnDelim(statusLine, "=");
            setIntegerParam(this->RBD9103_Range, getRangeSettingFromStr(range));
        } else if(statusLine.rfind("I, sample Interval", 0) == 0) {
            const char* sampleInterval = splitRespOnDelim(statusLine, "=").c_str();
            setIntegerParam(this->RBD9103_SamplingRate, atoi(sampleInterval));
        } else if(statusLine.rfind("L, Chart Log Update Interval", 0) == 0) {
            const char*chartLogUpdateInterval = splitRespOnDelim(statusLine, "=").c_str();
            //setStringParam(this->RBD9103_ChartLogUpdateInterval, );
        } else if(statusLine.rfind("B, BIAS", 0) == 0) {
            const char* bias = splitRespOnDelim(statusLine, "=").c_str();
            if(strcmp(bias, "ON") == 0){
                setIntegerParam(this->RBD9103_Bias, RBD_ON);
            } else if(strcmp(bias, "OFF") == 0){
                setIntegerParam(this->RBD9103_Bias, RBD_OFF);
            }
        } else if(statusLine.rfind("F, Filter", 0) == 0) {
            const char* filter = splitRespOnDelim(statusLine, "=").c_str();
            setIntegerParam(this->RBD9103_Filter, atoi(filter) / 2);
        } else if(statusLine.rfind("V, FormatLen", 0) == 0) {
            const char* formatLen = splitRespOnDelim(statusLine, "=").c_str();
            this->setIntegerParam(this->RBD9103_NumDigits, atoi(formatLen));
        } else if(statusLine.rfind("CA, Autocal", 0) == 0) {
            const char* autoCal = splitRespOnDelim(statusLine, "=").c_str();
            //setStringParam(this->RBD9103_AutoCal, );
        } else if(statusLine.rfind("G, AutoGrounding", 0) == 0) {
            const char* autoGrounding = splitRespOnDelim(statusLine, "=").c_str();
            if(strcmp(autoGrounding, "DISABLED") == 0){
                setIntegerParam(this->RBD9103_InputGnd, RBD_OFF);
            } else if(strcmp(autoGrounding, "ENABLED") == 0){
                setIntegerParam(this->RBD9103_InputGnd, RBD_ON);
            }
        } else if(statusLine.rfind("Q, State", 0) == 0) {
            string state = splitRespOnDelim(statusLine, "=");
            if(state == "MEASURE") {
                setIntegerParam(this->RBD9103_Sampling, RBD_ON);
            } else {
                setIntegerParam(this->RBD9103_Sampling, RBD_OFF);
            }
        } else if(statusLine.rfind("P, PID", 0) == 0) {
            const char* deviceId = splitRespOnDelim(statusLine, "=").c_str();
            setStringParam(this->RBD9103_DeviceID, deviceId);
            readDeviceID = true;
        }
    }
    callParamCallbacks();

    return asynSuccess;
}

RBDRange_t RBD9103::getRangeSettingFromStr(string rangeStr){
    if (rangeStr == "AutoR") {
        return RBD_RNG_AUTO;
    } else if (rangeStr == "002nA") {
        return RBD_RNG_2N;
    } else if (rangeStr == "020nA") {
        return RBD_RNG_20N;
    } else if (rangeStr == "200nA") {
        return RBD_RNG_200N;
    } else if (rangeStr == "002uA") {
        return RBD_RNG_2U;
    } else if (rangeStr == "020uA") {
        return RBD_RNG_20U;
    } else if (rangeStr == "200uA") {
        return RBD_RNG_200U;
    } else if (rangeStr == "002mA") {
        return RBD_RNG_2M;
    }
    return RBD_RNG_AUTO;
}

void RBD9103::parseSampling(string rawSampling){
    const char* functionName = "parseSampling";

    // Sampling response format:
    // &S=,Range=002nA,+0.0008,nA

    switch(rawSampling[2]){
        case '=':
            setIntegerParam(this->RBD9103_Stable, 1);
            break;
        default:
            setIntegerParam(this->RBD9103_Stable, 0);
            break;
    }

    string range = rawSampling.substr(rawSampling.find(",", 1) + 1, rawSampling.find(",", 2));
    range = splitRespOnDelim(range, "=");
    setIntegerParam(RBD9103_RangeActual, getRangeSettingFromStr(range));

    string current = rawSampling.substr(rawSampling.find(",", 2) + 1, rawSampling.find(",", 3));
    if (current[0] == '+'){
        current = current.substr(1, current.length());
    }
    setDoubleParam(this->RBD9103_Current, atof(current.c_str()));

    string units = rawSampling.substr(rawSampling.find(",", 3) + 1, rawSampling.length());
    setStringParam(this->RBD9103_Units, units.c_str());

    callParamCallbacks();
}


void RBD9103::sampleOnce(){
    const char* functionName = "sampleOnce";
    string sampling = writeReadCmd("&S");
    if (sampling == "") {
        ERR("Failed to read sampling response!");
        return;
    }
    parseSampling(sampling);
}


void RBD9103::samplingThread(){
    const char* functionName = "samplingThread";

    int sampling, samplingRateMs;
    char ret[64];
    size_t nread;
    int eomReason;
    asynStatus status;

    while(this->alive) {
        getIntegerParam(RBD9103_Sample, &sampling);
        if(sampling == 1){
            getIntegerParam(RBD9103_SamplingRate, &samplingRateMs);
            double samplingReadTO = samplingRateMs / 1000.0 + 0.1;
            asynStatus status = pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret), samplingReadTO, &nread, &eomReason);
            if (status != asynSuccess) {
                ERR("Failed to read sampling response!");
            } else {
                string samplingResp = string(ret);
                if(!samplingResp.rfind("&S", 0)){
                    ERR_ARGS("Recieved unexpected sampling response: %s... skipping!", samplingResp.c_str());
                } else{
                    parseSampling(samplingResp);
                }
            }
        }
    }
}


void RBD9103::setSamplingRate(int rate){
    const char* functionName = "setSamplingRate";
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "&I%04d", rate);
    writeReadCmd(cmd);
    this->getDeviceStatus();
}


asynStatus RBD9103::writeInt32(asynUser* pasynUser, epicsInt32 value){
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char* functionName = "writeInt32";
    char cmd[32], ret[64];
    if (function == RBD9103_Sample) {
        if(value == 0){
            this->setSamplingRate(0);
        } else {
            int samplingRateMs;
            getIntegerParam(RBD9103_SamplingRate, &samplingRateMs);
            if(samplingRateMs > 9999 || samplingRateMs < 20){
                ERR_ARGS("Invalid sampling rate: %d ms. Must be between 20 and 9999", samplingRateMs);
                status = asynError;
            } else {
                this->setSamplingRate(samplingRateMs);
            }
        }
    } else if (function == RBD9103_SampleOnce){
        int sampling;
        getIntegerParam(RBD9103_Sample, &sampling);
        if (sampling){
            ERR("Sampling is already started!");
            return asynError;
        } else {
            this->sampleOnce();
        }
    } else if (function == RBD9103_SamplingRate){
        if(value == 0) {
            setSamplingRate(0);
            getDeviceStatus();
        } else return asynSuccess;
    } else if (function == RBD9103_Bias) {
        snprintf(cmd, sizeof(cmd), "&B%d", value);
    } else if (function == RBD9103_Range) {
        snprintf(cmd, sizeof(cmd), "&R%d", value);
        // Setting range disables offset null
        setIntegerParam(RBD9103_OffsetNull, 0);
    } else if (function == RBD9103_Filter) {
        snprintf(cmd, sizeof(cmd), "&F%03d", value * 2);
    } else if (function == RBD9103_InputGnd) {
        snprintf(cmd, sizeof(cmd), "&G%d", value);
    } else if (function == RBD9103_OffsetNull) {
        int range;
        getIntegerParam(RBD9103_Range, &range);
        if(range == 0){
            ERR("Cannot set offset null when range is set to AutoR");
            status = asynError;
        } else if (value == 1) {
            snprintf(cmd, sizeof(cmd), "&N", value);
        } else {
            // Per manual, to clear offset null state, just re-apply range setting.
            snprintf(cmd, sizeof(cmd), "&R%d", range);
        }
    }

    if(strlen(cmd) > 0){
        writeReadCmd(cmd);
        getDeviceStatus();
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
    createParam(RBD9103_SamplingRateString, asynParamInt32, &this->RBD9103_SamplingRate);
    createParam(RBD9103_StableString, asynParamInt32, &this->RBD9103_Stable);
    createParam(RBD9103_SampleString, asynParamInt32, &this->RBD9103_Sample);
    createParam(RBD9103_SamplingString, asynParamInt32, &this->RBD9103_Sampling);
    createParam(RBD9103_UnitsString, asynParamInt32, &this->RBD9103_Units);
    createParam(RBD9103_CurrentString, asynParamFloat64, &this->RBD9103_Current);
    createParam(RBD9103_OffsetNullString, asynParamFloat64, &this->RBD9103_OffsetNull);
    createParam(RBD9103_InputGndString, asynParamInt32, &this->RBD9103_InputGnd);
    createParam(RBD9103_RangeString, asynParamInt32, &this->RBD9103_Range);
    createParam(RBD9103_RangeActualString, asynParamInt32, &this->RBD9103_RangeActual);
    createParam(RBD9103_SampleOnceString, asynParamInt32, &this->RBD9103_SampleOnce);
    createParam(RBD9103_BiasString, asynParamInt32, &this->RBD9103_Bias);
    createParam(RBD9103_FilterString, asynParamInt32, &this->RBD9103_Filter);
    createParam(RBD9103_AutoCalString, asynParamInt32, &this->RBD9103_AutoCal);
    createParam(RBD9103_AutoGroundingString, asynParamInt32, &this->RBD9103_AutoGrounding);
    createParam(RBD9103_DeviceIDString, asynParamOctet, &this->RBD9103_DeviceID);
    createParam(RBD9103_FwVersionString, asynParamOctet, &this->RBD9103_FwVersion);
    createParam(RBD9103_BuildDateString, asynParamOctet, &this->RBD9103_BuildDate);
    createParam(RBD9103_DeviceDescString, asynParamOctet, &this->RBD9103_DeviceDesc);
    createParam(RBD9103_ManufacturerString, asynParamOctet, &this->RBD9103_Manufacturer);

    setStringParam(this->RBD9103_DrvVersion, RBD9103_DRIVER_VERSION);
    string modelResp = writeReadCmd("&K");
    if (modelResp == "") {
        ERR("Failed to get model number");
        return;
    }

    // Response echos back &K and then follows with Key=$MODEL,
    // so split on the equal sign to get the model number alone.
    string model = modelResp.substr(modelResp.find("=") + 1, modelResp.length());

    setStringParam(this->RBD9103_Model, model.c_str());
    setStringParam(this->RBD9103_DeviceDesc, "PicoAmmeter");
    setStringParam(this->RBD9103_Manufacturer, "RBD Instruments");
    callParamCallbacks();

    LOG_ARGS("Connected to ammeter: %s", model.c_str());

    this->getDeviceStatus();

    epicsAtExit(exitCallbackC, (void*) this);
}

RBD9103::~RBD9103(){
    const char* functionName = "~RBD9103";

    LOG("Shutting down RBD 9103 IOC...");
    this->alive = false;
    if(this->samplingThreadId != NULL){
        epicsThreadMustJoin(this->samplingThreadId);
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
