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
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "WARN | %s::%s: %s\n", driverName, functionName, msg)

#define WARN_ARGS(fmt, ...)                                                            \
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "WARN | %s::%s: " fmt "\n", driverName, \
              functionName, __VA_ARGS__);

// Log message formatters
#define LOG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: %s\n", driverName, functionName, msg)

#define LOG_ARGS(fmt, ...)                                                                       \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: " fmt "\n", driverName, functionName, \
              __VA_ARGS__);

// Log message formatters
#define DEBUG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", driverName, functionName, msg)

#define DEBUG_ARGS(fmt, ...)                                                                       \
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: " fmt "\n", driverName, functionName, \
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


/**
 * Callback fired during a clean exit from the IOC application.
 * Cleans up sampling thread and destroys driver object.
*/
static void exitCallbackC(void* pPvt) {
    RBD9103* pRBD9103 = (RBD9103*)pPvt;
    delete pRBD9103;
}


/**
 * Wrapper function to allow for calling C++ class method from C function.
*/
static void samplingThreadC(void* pPvt){
    RBD9103* pRBD9103 = (RBD9103*)pPvt;
    pRBD9103->samplingThread();
}


/**
 * Helper function that sends command to low level serial port and waits for response
*/
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


string RBD9103::splitRespOnDelim(string resp, const char* delim){
    return resp.substr(resp.find(delim) + strlen(delim), resp.length());
}


/**
 * Helper function that reads current device status
 * 
 * Example output from &Q command
 * 
 *   Firmware Version: 02.09
 *   Build: 1-25-18
 *   R, Range=AutoR
 *   I, sample Interval=0500 mSec
 *   L, Chart Log Update Interval=0200 mSec
 *   B, BIAS=OFF
 *   F, Filter=032
 *   V, FormatLen=5
 *   CA, Autocal=OFF
 *   G, AutoGrounding=DISABLED
 *   Q, State=MEASURE
 *   P, PID=NEW_DEVICE
 * 
*/
void RBD9103::getDeviceStatus(){
    const char* functionName = "getDeviceStatus";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    size_t nwrite, nread;
    int eomReason;

    // Lock the mutex for the driver so we can guarantee that any reads are going to be coming from
    // the output of our command.
    this->lock();

    // Send &Q to get device status
    pasynOctetSyncIO->write(this->pasynUserSerialPort, "&Q", 2, 1, &nwrite);

    LOG("Reading device status:");
    bool readDeviceID = false;
    char ret[64];
    while(!readDeviceID){

        // Device status is several lines long, so we need to 
        pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret), 1, &nread, &eomReason);
        string statusLine = string(ret);
        LOG(statusLine.c_str());
        if(statusLine.rfind("Firmware", 0) == 0) {
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
            setIntegerParam(this->RBD9103_SamplingRateActual, atoi(sampleInterval));
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
            setIntegerParam(this->RBD9103_Filter, int(sqrt(atoi(filter))));
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
            setStringParam(RBD9103_State, splitRespOnDelim(statusLine, "=").c_str());
        } else if(statusLine.rfind("P, PID", 0) == 0) {
            const char* deviceId = splitRespOnDelim(statusLine, "=").c_str();
            setStringParam(this->RBD9103_DeviceID, deviceId);
            readDeviceID = true;
        }
    }
    // unlock to allow for aynchronous device access.
    this->unlock();
    callParamCallbacks();
}

void RBD9103::getModelNumber(){
    const char* functionName = "getModelNumber";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    size_t nwrite, nread;
    int eomReason;

    // Lock the mutex for the driver so we can guarantee that any reads are going to be coming from
    // the output of our command.
    this->lock();

    // Send &Q to get device status
    asynStatus status = pasynOctetSyncIO->write(this->pasynUserSerialPort, "&K", 2, 1, &nwrite);

    LOG("Reading device status:");
    bool readModelNumber = false;
    char ret[64];
    while(!readModelNumber){
        if(string(ret).rfind("&K, Key=")){
            const char* model = splitRespOnDelim(string(ret), "=").c_str();
            setStringParam(RBD9103_Model, model);
            LOG_ARGS("Connected to ammeter: %s", model);
            readModelNumber = true;
        } else {
            WARN("Recieved unexpected response for model number query. Trying again...");
        }
    }
    this->unlock();
    callParamCallbacks();
}


/**
 * Convert string representation of range setting to Enum
*/
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

static vector<string> split(string input, const char* delim) {
    vector<string> result;
    size_t pos = 0;
    string token;
    while ((pos = input.find(delim)) != string::npos) {
        token = input.substr(0, pos);
        result.push_back(token);
        input.erase(0, pos + strlen(delim));
    }
    result.push_back(input);
    return result;
}


/**
 * Parse single sampling response
*/
void RBD9103::parseSampling(string rawSampling){
    const char* functionName = "parseSampling";

    // Sampling response format:
    // &S=,Range=002nA,+0.0008,nA
    LOG_ARGS("Parsing sampling response: %s", rawSampling.c_str());

    int sampleCounter;
    getIntegerParam(RBD9103_SampleCounter, &sampleCounter);
    setIntegerParam(RBD9103_SampleCounter, sampleCounter + 1);

    vector<string> components = split(rawSampling, ",");
    for(int i = 0; i < components.size(); i++){
        switch(i) {
            case 0:
                switch(components[i][2]){
                    case '=':
                        setIntegerParam(this->RBD9103_Stable, 1);
                        break;
                    default:
                        setIntegerParam(this->RBD9103_Stable, 0);
                        break;
                }
                break;
            case 1:
            {
                string range = components[i].substr(components[i].find("=") + 1, components[i].length());
                setIntegerParam(this->RBD9103_Range, getRangeSettingFromStr(range));
                break;
            }
            case 2:
            {
                const char* current = components[i].c_str();
                if (current[0] == '+'){
                    current = current + 1;
                }
                setDoubleParam(this->RBD9103_Current, atof(current));
                break;
            }
            case 3:
            {
                const char* units = components[i].c_str();
                if (strcmp(units, "nA") == 0){
                    setIntegerParam(this->RBD9103_CurrentUnits, RBD_UNT_NA);
                } else if(strcmp(units, "uA") == 0) {
                    setIntegerParam(this->RBD9103_CurrentUnits, RBD_UNT_UA);
                } else if(strcmp(units, "mA") == 0) {
                    setIntegerParam(this->RBD9103_CurrentUnits, RBD_UNT_MA);
                } else {
                    ERR_ARGS("Recieved unexpected unit type: %s", units);
                }
                break;
            }
        }
    }


    callParamCallbacks();
}


/**
 * Main sampling loop. Runs on seperate thread spawned after connection is established.
*/
void RBD9103::samplingThread(){
    const char* functionName = "samplingThread";

    int sampling, samplingRateMs;
    char ret[64];
    size_t nread;
    int eomReason;
    int numSamples, sampleCounter;
    RBDSamplingMode_t samplingMode;
    asynStatus status;

    while(this->alive) {
        getIntegerParam(RBD9103_Sample, &sampling);
        getIntegerParam(RBD9103_SamplingRateActual, &samplingRateMs);
        getIntegerParam(RBD9103_SamplingMode, (int*) &samplingMode);
        if(sampling == 1 && samplingMode != RBD_SAMPLING_MODE_SINGLE && samplingRateMs >= 20 && samplingRateMs <= 9999){
            getIntegerParam(RBD9103_NumSamples, &numSamples);

            double samplingReadTO = samplingRateMs / 1000.0 + 0.1;
            this->lock();
            asynStatus status = pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret), samplingReadTO, &nread, &eomReason);
            this->unlock();
            if (status != asynSuccess) {
                ERR("Failed to read sampling response!");
            } else {
                if(ret[0] != '&' || ret[1] != 'S'){
                    ERR_ARGS("Recieved unexpected sampling response: %s... skipping!", ret);
                } else{
                    string samplingResp = string(ret);
                    parseSampling(samplingResp);
                    getIntegerParam(RBD9103_SampleCounter, &sampleCounter);
                    if(samplingMode == RBD_SAMPLING_MODE_MULTIPLE && sampleCounter == numSamples){
                        setIntegerParam(RBD9103_Sample, 0);
                    }
                }
            }
            callParamCallbacks();
        } else {
            epicsThreadSleep(0.05);
        }

    } 
}

void RBD9103::startSampling(){
    this->alive = true;

    epicsThreadOpts opts;
    opts.priority = epicsThreadPriorityMedium;
    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
    opts.joinable = 1;

    // Spawn our sampling thread. Make it joinable
    this->samplingThreadId =
        epicsThreadCreateOpt("samplingThread", (EPICSTHREADFUNC) samplingThreadC, this, &opts);
}

void RBD9103::stopSampling(){
    this->alive = false;
    if(this->samplingThreadId != NULL){
        epicsThreadMustJoin(this->samplingThreadId);
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
        RBDSamplingMode_t samplingMode;
        getIntegerParam(RBD9103_SamplingMode, (int*) &samplingMode);

        if(value == 0){
            this->setSamplingRate(0);
        } else if(samplingMode == RBD_SAMPLING_MODE_SINGLE) {
            string samplingResp = writeReadCmd("&S");
            parseSampling(samplingResp);
            setIntegerParam(RBD9103_Sample, 0);
        } else {
            int samplingRateMs;
            getIntegerParam(RBD9103_SamplingRate, &samplingRateMs);
            if(samplingRateMs > 9999 || samplingRateMs < 20){
                ERR_ARGS("Invalid sampling rate: %d ms. Must be between 20 and 9999", samplingRateMs);
                status = asynError;
            } else {
                setIntegerParam(RBD9103_SampleCounter, 0);
                this->setSamplingRate(samplingRateMs);
                this->startSampling();
            }
        }
    } else if (function == RBD9103_SamplingRate){
        if(value == 0) {
            setSamplingRate(0);
            getDeviceStatus();
        }
    } else if (function == RBD9103_Bias) {
        snprintf(cmd, sizeof(cmd), "&B%d", value);
    } else if (function == RBD9103_Range) {
        snprintf(cmd, sizeof(cmd), "&R%d", value);
        // Setting range disables offset null
        setIntegerParam(RBD9103_OffsetNull, 0);
    } else if (function == RBD9103_Filter) {
        snprintf(cmd, sizeof(cmd), "&F%03d", pow(value, 2));
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

    if(strlen(cmd) > 0) {
        writeReadCmd(cmd);
        getDeviceStatus();
    }

    if (status) {
        ERR_ARGS("ERROR status=%d, function=%d, value=%d", status, function, value);
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

    // Connect this driver to the low level serial port, so we can access it later.
    pasynOctetSyncIO->connect(serialPortName, 0, &this->pasynUserSerialPort, NULL);

    createParam(RBD9103_ModelString, asynParamOctet, &this->RBD9103_Model);
    createParam(RBD9103_DrvVersionString, asynParamOctet, &this->RBD9103_DrvVersion);
    createParam(RBD9103_NumDigitsString, asynParamInt32, &this->RBD9103_NumDigits);
    createParam(RBD9103_SamplingRateString, asynParamInt32, &this->RBD9103_SamplingRate);
    createParam(RBD9103_StableString, asynParamInt32, &this->RBD9103_Stable);
    createParam(RBD9103_SampleString, asynParamInt32, &this->RBD9103_Sample);
    createParam(RBD9103_SamplingRateActualString, asynParamInt32, &this->RBD9103_SamplingRateActual);
    createParam(RBD9103_SamplingModeString, asynParamInt32, &this->RBD9103_SamplingMode);
    createParam(RBD9103_InRangeString, asynParamInt32, &this->RBD9103_InRange);
    createParam(RBD9103_NumSamplesString, asynParamInt32, &this->RBD9103_NumSamples);
    createParam(RBD9103_SampleCounterString, asynParamInt32, &this->RBD9103_SampleCounter);
    createParam(RBD9103_StateString, asynParamOctet, &this->RBD9103_State);
    createParam(RBD9103_CurrentUnitsString, asynParamInt32, &this->RBD9103_CurrentUnits);
    createParam(RBD9103_CurrentString, asynParamFloat64, &this->RBD9103_Current);
    createParam(RBD9103_OffsetNullString, asynParamFloat64, &this->RBD9103_OffsetNull);
    createParam(RBD9103_InputGndString, asynParamInt32, &this->RBD9103_InputGnd);
    createParam(RBD9103_RangeString, asynParamInt32, &this->RBD9103_Range);
    createParam(RBD9103_RangeActualString, asynParamInt32, &this->RBD9103_RangeActual);
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

    // Get the model number
    this->getModelNumber();

    setStringParam(this->RBD9103_DeviceDesc, "PicoAmmeter");
    setStringParam(this->RBD9103_Manufacturer, "RBD Instruments");

    this->getDeviceStatus();
    setIntegerParam(this->RBD9103_OffsetNull, 0);

    // Make sure we flush any parameter settings, after fetching status.
    callParamCallbacks();

    epicsAtExit(exitCallbackC, (void*) this);
}

RBD9103::~RBD9103(){
    const char* functionName = "~RBD9103";

    // Make sure we clean up our sampling thread here.
    LOG("Shutting down RBD 9103 IOC...");
    if(this->samplingThreadId != NULL){
        this->stopSampling();
    }
    LOG("Done");
}



//-------------------------------------------------------------
// RBD9103 ioc shell registration
//-------------------------------------------------------------

// port name of the driver itself
static const iocshArg RBD9103ConfigArg0 = {"portName", iocshArgString};

// port name of the configured low-level serial port connected to the device
static const iocshArg RBD9103ConfigArg1 = {"serialPortNameName", iocshArgString};

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
