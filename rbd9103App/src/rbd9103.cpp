// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cmath>

// EPICS includes
#include <asynOctetSyncIO.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <inttypes.h>
#include <iocsh.h>

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
#define WARN(msg)                                                                                 \
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "WARN | %s::%s: %s\n", driverName, functionName, \
              msg)

#define WARN_ARGS(fmt, ...)                                                              \
    asynPrint(pasynUserSelf, ASYN_TRACE_WARNING, "WARN | %s::%s: " fmt "\n", driverName, \
              functionName, __VA_ARGS__);

// Log message formatters
#define DEBUG(msg) \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: %s\n", driverName, functionName, msg)

#define DEBUG_ARGS(fmt, ...)                                                                     \
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER, "%s::%s: " fmt "\n", driverName, functionName, \
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
    RBD9103* pRBD9103 = (RBD9103*) pPvt;
    delete pRBD9103;
}

/**
 * Wrapper function to allow for calling C++ class method from C function.
 */
static void samplingThreadC(void* pPvt) {
    RBD9103* pRBD9103 = (RBD9103*) pPvt;
    pRBD9103->samplingThread();
}

/**
 * Helper function that sends command to low level serial port and waits for response
 */
string RBD9103::writeReadCmd(const char* cmd) {
    size_t nwrite, nread;
    asynStatus status;
    int eomReason;
    char ret[64];

    const char* functionName = "writeReadCmd";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    status = pasynOctetSyncIO->writeRead(this->pasynUserSerialPort, cmd, strlen(cmd), ret, 64,
                                         TIMEOUT, &nwrite, &nread, &eomReason);

    if (status != asynSuccess) {
        ERR("Failed to write command!");
        return string("");
    }
    return string(ret);
}

string RBD9103::splitRespOnDelim(string resp, const char* delim) {
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
void RBD9103::getDeviceStatus() {
    const char* functionName = "getDeviceStatus";
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);
    size_t nwrite, nread;
    int eomReason;

    // Lock the mutex for the driver so we can guarantee that any reads are going to be coming from
    // the output of our command.
    this->lock();

    // Send &Q to get device status
    pasynOctetSyncIO->write(this->pasynUserSerialPort, "&Q", 2, 1, &nwrite);

    DEBUG("Reading device status:");
    bool readDeviceID = false;
    char ret[64];
    while (!readDeviceID) {
        // Device status is several lines long, so we need to
        pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret), 1, &nread, &eomReason);
        string statusLine = string(ret);
        DEBUG(statusLine.c_str());
        if (statusLine.rfind("Firmware", 0) == 0) {
            const char* fwVersion = splitRespOnDelim(statusLine, ": ").c_str();
            setStringParam(this->RBD9103_FwVersion, fwVersion);
        } else if (statusLine.rfind("Build", 0) == 0) {
            const char* buildDate = splitRespOnDelim(statusLine, ": ").c_str();
            setStringParam(this->RBD9103_BuildDate, buildDate);
        } else if (statusLine.rfind("R, Range", 0) == 0) {
            string range = splitRespOnDelim(statusLine, "=");
            // If range is not set to AutoR, update range actual.
            // Otherwise, range actual will be set in sampling loop.
            if (range != "AutoR")
                setIntegerParam(this->RBD9103_RangeActual, getRangeSettingFromStr(range));
            setIntegerParam(this->RBD9103_Range, getRangeSettingFromStr(range));
        } else if (statusLine.rfind("I, sample Interval", 0) == 0) {
            const char* sampleInterval = splitRespOnDelim(statusLine, "=").c_str();
            setIntegerParam(this->RBD9103_SamplingRateActual, atoi(sampleInterval));
        } else if (statusLine.rfind("B, BIAS", 0) == 0) {
            const char* bias = splitRespOnDelim(statusLine, "=").c_str();
            if (strcmp(bias, "ON") == 0) {
                setIntegerParam(this->RBD9103_Bias, RBD_ON);
            } else if (strcmp(bias, "OFF") == 0) {
                setIntegerParam(this->RBD9103_Bias, RBD_OFF);
            }
        } else if (statusLine.rfind("F, Filter", 0) == 0) {
            const char* filter = splitRespOnDelim(statusLine, "=").c_str();
            setIntegerParam(this->RBD9103_Filter, int(log2(atoi(filter))));
        } else if (statusLine.rfind("V, FormatLen", 0) == 0) {
            const char* formatLen = splitRespOnDelim(statusLine, "=").c_str();
            this->setIntegerParam(this->RBD9103_NumDigits, atoi(formatLen));
        } else if (statusLine.rfind("CA, Autocal", 0) == 0) {
            const char* autoCal = splitRespOnDelim(statusLine, "=").c_str();
            if (strcmp(autoCal, "ON") == 0) {
                setIntegerParam(this->RBD9103_AutoCal, RBD_ON);
            } else if (strcmp(autoCal, "OFF") == 0) {
                setIntegerParam(this->RBD9103_AutoCal, RBD_OFF);
            }
        } else if (statusLine.rfind("G, AutoGrounding", 0) == 0) {
            const char* autoGrounding = splitRespOnDelim(statusLine, "=").c_str();
            if (strcmp(autoGrounding, "DISABLED") == 0) {
                setIntegerParam(this->RBD9103_InputGnd, RBD_OFF);
            } else if (strcmp(autoGrounding, "ENABLED") == 0) {
                setIntegerParam(this->RBD9103_InputGnd, RBD_ON);
            }
        } else if (statusLine.rfind("Q, State", 0) == 0) {
            setStringParam(RBD9103_State, splitRespOnDelim(statusLine, "=").c_str());
        } else if (statusLine.rfind("P, PID", 0) == 0) {
            const char* deviceId = splitRespOnDelim(statusLine, "=").c_str();
            setStringParam(this->RBD9103_DeviceID, deviceId);
            readDeviceID = true;
        }
    }
    // unlock to allow for aynchronous device access.
    this->unlock();
    callParamCallbacks();
}

void RBD9103::getModelNumber() {
    pasynOctetSyncIO->flush(this->pasynUserSerialPort);

    // Lock the mutex for the driver so we can guarantee that any reads are going to be coming from
    // the output of our command.
    this->lock();

    string modelStr = writeReadCmd("&K");

    const char* model = splitRespOnDelim(modelStr, "=").c_str();
    setStringParam(RBD9103_Model, model);
    printf("Connected to ammeter: %s\n", model);

    this->unlock();
    callParamCallbacks();
}

/**
 * Convert string representation of range setting to Enum
 */
RBDRange_t RBD9103::getRangeSettingFromStr(string rangeStr) {
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

const char* RBD9103::getStrFromRangeSetting(RBDRange_t range) {
    switch (range) {
        case RBD_RNG_AUTO:
            return "AutoR";
        case RBD_RNG_2N:
            return "002nA";
        case RBD_RNG_20N:
            return "020nA";
        case RBD_RNG_200N:
            return "200nA";
        case RBD_RNG_2U:
            return "002uA";
        case RBD_RNG_20U:
            return "020uA";
        case RBD_RNG_200U:
            return "200uA";
        case RBD_RNG_2M:
            return "002mA";
    }
    return "";
}

RBDUnits_t RBD9103::getUnitsFromStr(string unitsStr) {
    if (unitsStr == "nA") {
        return RBD_UNT_NA;
    } else if (unitsStr == "uA") {
        return RBD_UNT_UA;
    } else if (unitsStr == "mA") {
        return RBD_UNT_MA;
    }
    return RBD_UNT_NA;
}

const char* RBD9103::getStrFromUnits(RBDUnits_t units) {
    switch (units) {
        case RBD_UNT_NA:
            return "nA";
        case RBD_UNT_UA:
            return "uA";
        case RBD_UNT_MA:
            return "mA";
    }
    return "";
}

static uint64_t getTimestampMS() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t) ts.tv_sec * 1000 + (uint64_t) ts.tv_nsec / 1000000;
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

void RBD9103::appendCSV(double current, double avgCurrent, RBDRange_t range,
                        RBDUnits_t currentUnits, int stable) {
    const char* functionName = "appendCSV";
    if (this->csvFile != NULL) {
        fprintf(this->csvFile, "%" PRIu64 ",%d,%s,%f,%f,%s\n", getTimestampMS(), stable,
                getStrFromRangeSetting(range), current, avgCurrent, getStrFromUnits(currentUnits));
        fflush(this->csvFile);
    } else {
        ERR("CSV file not open!");
    }
}

void RBD9103::openCSV(const char* filename) {
    const char* functionName = "openCSV";
    if (this->csvFile != NULL) {
        fclose(this->csvFile);
    }
    this->csvFile = fopen(filename, "w");
    if (this->csvFile == NULL) {
        ERR_ARGS("Failed to open CSV file: %s", filename);
    } else {
        fprintf(this->csvFile, "Timestamp,Stable,Range,Current,AvgCurrent,Units\n");
        fflush(this->csvFile);
        printf("Opened CSV file: %s\n", filename);
    }
}

void RBD9103::closeCSV() {
    if (this->csvFile != NULL) {
        fclose(this->csvFile);
        printf("Closed CSV file\n");
    }
    this->csvFile = NULL;
}

/**
 * Parse single sampling response
 */
void RBD9103::parseSampling(string rawSampling) {
    const char* functionName = "parseSampling";

    RBDUnits_t avgCurrentUnits, currentUnits = RBD_UNT_NA;
    RBDOnOff_t avgOnlyStable = RBD_ON;
    RBDRange_t range = RBD_RNG_2N;
    double avgCurrent, sumCurrent = 0;
    getDoubleParam(RBD9103_SumCurrent, &sumCurrent);
    getIntegerParam(RBD9103_AvgOnlyStable, (int*) &avgOnlyStable);
    getIntegerParam(RBD9103_AvgCurrentUnits, (int*) &avgCurrentUnits);

    DEBUG_ARGS("Parsing sampling response: %s", rawSampling.c_str());

    int stable = 0;
    int inRange = 0;
    double current = 0;

    // Sampling response format:
    // &S=,Range=002nA,+0.0008,nA
    vector<string> components = split(rawSampling, ",");
    for (size_t i = 0; i < components.size(); i++) {
        switch (i) {
            case 0:
                switch (components[i][2]) {
                    case '=':
                        stable = 1;
                        inRange = RBD_RANGE_STATE_OK;
                        break;
                    case '>':
                        inRange = RBD_RANGE_STATE_OVER;
                        break;
                    case '<':
                        inRange = RBD_RANGE_STATE_UNDER;
                        break;
                    default:
                        inRange = RBD_RANGE_STATE_OK;
                        break;
                }

                setIntegerParam(this->RBD9103_Stable, stable);
                setIntegerParam(this->RBD9103_InRange, inRange);
                break;
            case 1: {
                string rangeStr =
                    components[i].substr(components[i].find("=") + 1, components[i].length());
                range = getRangeSettingFromStr(rangeStr);

                // Only update range readback if it is not set to Auto.
                RBDRange_t rangeSetting;
                getIntegerParam(RBD9103_Range, (int*) &rangeSetting);
                if (rangeSetting != RBD_RNG_AUTO) {
                    setIntegerParam(this->RBD9103_Range, range);
                }

                // Always update range actual, even if it is AutoR.
                setIntegerParam(this->RBD9103_RangeActual, range);
                break;
            }
            case 2: {
                const char* currentStr = components[i].c_str();
                if (currentStr[0] == '+') {
                    currentStr = currentStr + 1;
                }
                current = atof(currentStr);
                setDoubleParam(this->RBD9103_Current, current);
                break;
            }
            case 3: {
                string unitsStr = components[i].c_str();
                currentUnits = getUnitsFromStr(unitsStr);
                setIntegerParam(this->RBD9103_CurrentUnits, currentUnits);
                break;
            }
        }
    }

    // Only perform averaging/saving if stable or explicitly told do
    // average everything.
    if (stable == 1 || avgOnlyStable == RBD_OFF) {
        // Increment our sample counter
        int sampleCounter;
        getIntegerParam(RBD9103_SampleCounter, &sampleCounter);
        sampleCounter++;
        setIntegerParam(RBD9103_SampleCounter, sampleCounter);

        // Scale current to the correct units. For example, if current reading is 0.0008 nA,
        // and we are requesting an mA scale, then we need to divide by 1000000. To do this,
        // we can take the int representation of nA (0) and subtract the int representation of mA
        // (2) and then multiply our current by 1000 to that power.
        int unitsMultiplier = (int) currentUnits - (int) avgCurrentUnits;
        double scaledCurrent = current * pow(1000, unitsMultiplier);

        sumCurrent += scaledCurrent;
        avgCurrent = sumCurrent / (double) sampleCounter;
        setDoubleParam(RBD9103_AvgCurrent, avgCurrent);
        setDoubleParam(RBD9103_SumCurrent, sumCurrent);

        // Append to CSV file, if it is open
        RBDOnOff_t recording;
        getIntegerParam(RBD9103_Record, (int*) &recording);
        if (recording == RBD_ON) {
            appendCSV(current, avgCurrent, range, currentUnits, stable);
        }
    } else {
        WARN("Skipping average calculation and saving to csv file because sample is not stable");
    }

    callParamCallbacks();
}

/**
 * Main sampling loop. Runs on seperate thread spawned after connection is established.
 */
void RBD9103::samplingThread() {
    const char* functionName = "samplingThread";

    int sampling, samplingRateMs;
    char ret[64];
    size_t nread;
    int eomReason;
    int numSamples, sampleCounter;
    RBDSamplingMode_t samplingMode;

    setDoubleParam(RBD9103_SumCurrent, 0.0);
    setDoubleParam(RBD9103_AvgCurrent, 0.0);
    setIntegerParam(RBD9103_SampleCounter, 0);

    while (this->alive) {
        getIntegerParam(RBD9103_Sample, &sampling);
        getIntegerParam(RBD9103_SamplingRateActual, &samplingRateMs);
        getIntegerParam(RBD9103_SamplingMode, (int*) &samplingMode);

        if (sampling == 1 && samplingMode != RBD_SAMPLING_MODE_SINGLE && samplingRateMs >= 20 &&
            samplingRateMs <= 9999) {
            getIntegerParam(RBD9103_NumSamples, &numSamples);

            double samplingReadTO = samplingRateMs / 1000.0 + 0.1;
            this->lock();
            asynStatus status = pasynOctetSyncIO->read(this->pasynUserSerialPort, ret, sizeof(ret),
                                                       samplingReadTO, &nread, &eomReason);
            this->unlock();
            if (status != asynSuccess) {
                ERR("Failed to read sampling response!");
            } else {
                if (ret[0] != '&' || ret[1] != 'S') {
                    ERR_ARGS("Recieved unexpected sampling response: %s... skipping!", ret);
                } else {
                    string samplingResp = string(ret);
                    parseSampling(samplingResp);

                    getIntegerParam(RBD9103_SampleCounter, &sampleCounter);

                    if (samplingMode == RBD_SAMPLING_MODE_MULTIPLE && sampleCounter >= numSamples) {
                        setIntegerParam(RBD9103_Sample, RBD_OFF);
                        setIntegerParam(RBD9103_Record, RBD_OFF);
                        this->closeCSV();
                        this->alive = false;
                    }
                }
            }
            callParamCallbacks();
        } else {
            epicsThreadSleep(0.05);
        }
    }
}

void RBD9103::startSampling() {
    this->alive = true;

    epicsThreadOpts opts;
    opts.priority = epicsThreadPriorityMedium;
    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
    opts.joinable = 1;

    // Spawn our sampling thread. Make it joinable
    this->samplingThreadId =
        epicsThreadCreateOpt("samplingThread", (EPICSTHREADFUNC) samplingThreadC, this, &opts);
}

void RBD9103::stopSampling() {
    this->alive = false;
    if (this->samplingThreadId != NULL) {
        epicsThreadMustJoin(this->samplingThreadId);
    }
}

void RBD9103::setSamplingRate(int rate) {
    const char* functionName = "setSamplingRate";
    char cmd[32];
    DEBUG_ARGS("Setting sampling rate to %d ms", rate);
    snprintf(cmd, sizeof(cmd), "&I%04d", rate);
    writeReadCmd(cmd);
    this->getDeviceStatus();
}

asynStatus RBD9103::writeOctet(asynUser* pasynUser, const char* value, size_t nChars,
                               size_t* nActual) {
    const char* functionName = "writeOctet";
    asynStatus status = asynSuccess;
    int function = pasynUser->reason;
    int addr = 0;

    status = getAddress(pasynUser, &addr);
    if (status != asynSuccess) return (status);
    // Set the parameter in the parameter library.
    status = (asynStatus) setStringParam(addr, function, (char*) value);

    if (function == RBD9103_DirectoryPath && nChars > 0) {
        // Check if the directory exists and is writable
        struct stat info;
        DEBUG_ARGS("Checking if path %s exists and is writable", value);

        if (stat(value, &info) != 0 || !(info.st_mode & S_IFDIR)) {
            ERR_ARGS("Path %s does not exist or is not a directory", value);
            setIntegerParam(RBD9103_DirExistsAndWritable, 0);
            status = asynError;
        } else if (access(value, W_OK) != 0) {
            ERR_ARGS("Path %s is not writable", value);
            setIntegerParam(RBD9103_DirExistsAndWritable, 0);
            status = asynError;
        } else {
            setStringParam(RBD9103_DirectoryPath, value);
            setIntegerParam(RBD9103_DirExistsAndWritable, 1);
        }
    } else {
        setStringParam(function, value);
    }

    callParamCallbacks(addr);
    *nActual = nChars;
    return status;
}

asynStatus RBD9103::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char* functionName = "writeInt32";
    char cmd[32];
    if (function == RBD9103_Sample) {
        RBDSamplingMode_t samplingMode;
        getIntegerParam(RBD9103_SamplingMode, (int*) &samplingMode);

        if (value == 0) {
            this->setSamplingRate(0);
        } else if (samplingMode == RBD_SAMPLING_MODE_SINGLE) {
            string samplingResp = writeReadCmd("&S");
            parseSampling(samplingResp);
            setIntegerParam(RBD9103_Sample, 0);
            return asynSuccess;
        } else {
            int samplingRateMs;
            getIntegerParam(RBD9103_SamplingRate, &samplingRateMs);
            if (samplingRateMs > 9999 || samplingRateMs < 20) {
                ERR_ARGS("Invalid sampling rate: %d ms. Must be between 20 and 9999",
                         samplingRateMs);
                status = asynError;
            } else {
                this->setSamplingRate(samplingRateMs);
                this->startSampling();
            }
        }
    } else if (function == RBD9103_Record) {
        if (value == 0) {
            this->closeCSV();
        } else {
            int filePathExists;
            getIntegerParam(RBD9103_DirExistsAndWritable, &filePathExists);
            if (filePathExists == 1) {
                char filePath[256], fileName[256];
                getStringParam(RBD9103_FileName, 256, fileName);
                getStringParam(RBD9103_DirectoryPath, 256, filePath);
                char fullFilePath[512];
                snprintf(fullFilePath, sizeof(fullFilePath), "%s/%s", filePath, fileName);
                openCSV(fullFilePath);
            } else {
                ERR("Cannot start recording without a valid file path!");
                status = asynError;
            }
        }
    } else if (function == RBD9103_AvgOnlyStable || function == RBD9103_AvgCurrentUnits) {
        int sampling;
        getIntegerParam(RBD9103_Sample, &sampling);
        if (sampling == 1) {
            ERR("Cannot change averaging units or whether to only average stable samples while "
                "sampling is active!");
            status = asynError;
        }
    } else if (function == RBD9103_SamplingRate) {
        if (value == 0) {
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
        snprintf(cmd, sizeof(cmd), "&F%03d", int(pow(2, value)));
    } else if (function == RBD9103_InputGnd) {
        snprintf(cmd, sizeof(cmd), "&G%d", value);
    } else if (function == RBD9103_OffsetNull) {
        int range;
        getIntegerParam(RBD9103_Range, &range);
        if (range == 0) {
            ERR("Cannot set offset null when range is set to AutoR");
            status = asynError;
        } else if (value == 1) {
            snprintf(cmd, sizeof(cmd), "&N");
        } else {
            // Per manual, to clear offset null state, just re-apply range setting.
            snprintf(cmd, sizeof(cmd), "&R%d", range);
        }
    }

    if (strlen(cmd) > 0) {
        DEBUG_ARGS("Sending command: %s", cmd);
        writeReadCmd(cmd);
        getDeviceStatus();
    }

    if (status) {
        ERR_ARGS("ERROR status=%d, function=%d, value=%d", status, function, value);
    } else {
        setIntegerParam(function, value);
        DEBUG_ARGS("function=%d value=%d", function, value);
    }

    callParamCallbacks();
    return status;
}

RBD9103::RBD9103(const char* portName, const char* serialPortName)
    : asynPortDriver(
          portName, 1, /* maxAddr */
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

    printf("Initializing RBD9103 driver...\n");
    DEBUG_ARGS("Driver version: %s", RBD9103_DRIVER_VERSION);

    // Connect this driver to the low level serial port, so we can access it later.
    pasynOctetSyncIO->connect(serialPortName, 0, &this->pasynUserSerialPort, NULL);

    createParam(RBD9103_ModelString, asynParamOctet, &this->RBD9103_Model);
    createParam(RBD9103_DrvVersionString, asynParamOctet, &this->RBD9103_DrvVersion);
    createParam(RBD9103_NumDigitsString, asynParamInt32, &this->RBD9103_NumDigits);
    createParam(RBD9103_SamplingRateString, asynParamInt32, &this->RBD9103_SamplingRate);
    createParam(RBD9103_StableString, asynParamInt32, &this->RBD9103_Stable);
    createParam(RBD9103_SampleString, asynParamInt32, &this->RBD9103_Sample);
    createParam(RBD9103_SamplingRateActualString, asynParamInt32,
                &this->RBD9103_SamplingRateActual);
    createParam(RBD9103_SamplingModeString, asynParamInt32, &this->RBD9103_SamplingMode);
    createParam(RBD9103_InRangeString, asynParamInt32, &this->RBD9103_InRange);
    createParam(RBD9103_NumSamplesString, asynParamInt32, &this->RBD9103_NumSamples);
    createParam(RBD9103_SampleCounterString, asynParamInt32, &this->RBD9103_SampleCounter);
    createParam(RBD9103_StateString, asynParamOctet, &this->RBD9103_State);
    createParam(RBD9103_AutoCalString, asynParamInt32, &this->RBD9103_AutoCal);
    createParam(RBD9103_CurrentUnitsString, asynParamInt32, &this->RBD9103_CurrentUnits);
    createParam(RBD9103_CurrentString, asynParamFloat64, &this->RBD9103_Current);
    createParam(RBD9103_OffsetNullString, asynParamInt32, &this->RBD9103_OffsetNull);
    createParam(RBD9103_InputGndString, asynParamInt32, &this->RBD9103_InputGnd);
    createParam(RBD9103_RangeString, asynParamInt32, &this->RBD9103_Range);
    createParam(RBD9103_RangeActualString, asynParamInt32, &this->RBD9103_RangeActual);
    createParam(RBD9103_BiasString, asynParamInt32, &this->RBD9103_Bias);
    createParam(RBD9103_FilterString, asynParamInt32, &this->RBD9103_Filter);
    createParam(RBD9103_AutoGroundingString, asynParamInt32, &this->RBD9103_AutoGrounding);
    createParam(RBD9103_DeviceIDString, asynParamOctet, &this->RBD9103_DeviceID);
    createParam(RBD9103_FwVersionString, asynParamOctet, &this->RBD9103_FwVersion);
    createParam(RBD9103_BuildDateString, asynParamOctet, &this->RBD9103_BuildDate);
    createParam(RBD9103_AvgCurrentString, asynParamFloat64, &this->RBD9103_AvgCurrent);
    createParam(RBD9103_AvgCurrentUnitsString, asynParamInt32, &this->RBD9103_AvgCurrentUnits);
    createParam(RBD9103_AvgOnlyStableString, asynParamInt32, &this->RBD9103_AvgOnlyStable);
    createParam(RBD9103_SumCurrentString, asynParamFloat64, &this->RBD9103_SumCurrent);
    createParam(RBD9103_DeviceDescString, asynParamOctet, &this->RBD9103_DeviceDesc);
    createParam(RBD9103_DirectoryPathString, asynParamOctet, &this->RBD9103_DirectoryPath);
    createParam(RBD9103_FileNameString, asynParamOctet, &this->RBD9103_FileName);
    createParam(RBD9103_RecordString, asynParamInt32, &this->RBD9103_Record);
    createParam(RBD9103_DirExistsAndWritableString, asynParamInt32,
                &this->RBD9103_DirExistsAndWritable);
    createParam(RBD9103_ManufacturerString, asynParamOctet, &this->RBD9103_Manufacturer);

    setStringParam(this->RBD9103_DrvVersion, RBD9103_DRIVER_VERSION);

    // Get the model number
    this->getModelNumber();

    setStringParam(this->RBD9103_DeviceDesc, "PicoAmmeter");
    setStringParam(this->RBD9103_Manufacturer, "RBD Instruments");

    this->getDeviceStatus();

    // Make sure we flush any parameter settings, after fetching status.
    callParamCallbacks();

    epicsAtExit(exitCallbackC, (void*) this);
}

RBD9103::~RBD9103() {
    // Make sure we clean up our sampling thread here.
    printf("Shutting down RBD 9103 IOC...\n");

    if (this->csvFile != NULL) {
        this->closeCSV();
    }

    if (this->samplingThreadId != NULL) {
        this->stopSampling();
    }
    printf("Done.\n");
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
