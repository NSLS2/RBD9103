#ifndef RBD9103_H
#define RBD9103_H

#define RBD9103_VERSION      0
#define RBD9103_REVISION     0
#define RBD9103_MODIFICATION 0


#include <vector>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <epicsExit.h>
#include <iocsh.h>
#include <epicsExport.h>

#include <asynPortDriver.h>

#define RBD9103_DRIVER_VERSION "0.0.1"

static const char* driverName = "RBD9103";

#define RBD9103_ModelString "RBD_MODEL"
#define RBD9103_DrvVersionString "RBD_DRV_VERSION"
#define RBD9103_NumDigitsString "RBD_NUM_DIGITS"
#define RBD9103_SamplingRateString "RBD_SAMPLING_RATE"
#define RBD9103_SamplingRateActualString "RBD_SAMPLING_RATE_ACTUAL"
#define RBD9103_StateString "RBD_STATE"
#define RBD9103_StableString "RBD_STABLE"
#define RBD9103_SampleString "RBD_SAMPLE"
#define RBD9103_SamplingString "RBD_SAMPLING"
#define RBD9103_CurrentUnitsString "RBD_CURRENT_UNITS"
#define RBD9103_CurrentString "RBD_CURRENT"
#define RBD9103_RangeString "RBD_RANGE"
#define RBD9103_RangeActualString "RBD_RANGE_ACTUAL"
#define RBD9103_InRangeString "RBD_IN_RANGE"
#define RBD9103_SamplingModeString "RBD_SAMPLING_MODE"
#define RBD9103_NumSamplesString "RBD_NUM_SAMPLES"
#define RBD9103_SampleCounterString "RBD_SAMPLE_COUNTER"
#define RBD9103_OffsetNullString "RBD_OFFSET_NULL"
#define RBD9103_BiasString "RBD_BIAS"
#define RBD9103_FilterString "RBD_FILTER"
#define RBD9103_AutoCalString "RBD_AUTO_CAL"
#define RBD9103_AutoGroundingString "RBD_AUTO_GROUNDING"
#define RBD9103_DeviceIDString "RBD_DEVICE_ID"
#define RBD9103_FwVersionString "RBD_FW_VERSION"
#define RBD9103_BuildDateString "RBD_BUILD_DATE"
#define RBD9103_DeviceDescString "RBD_DEVICE_DESC"
#define RBD9103_ManufacturerString "RBD_MANUFACTURER"
#define RBD9103_AvgCurrentString "RBD_AVG_CURRENT"
#define RBD9103_AvgCurrentUnitsString "RBD_AVG_CURRENT_UNITS"
#define RBD9103_AvgOnlyStableString "RBD_AVG_ONLY_STABLE"
#define RBD9103_SumCurrentString "RBD_SUM_CURRENT"
#define RBD9103_RecordString "RBD_RECORD"
#define RBD9103_DirectoryPathString "RBD_DIRECTORY_PATH"
#define RBD9103_FileNameString "RBD_FILENAME"
#define RBD9103_DirExistsAndWritableString "RBD_DIR_EXISTS_WRITABLE"
#define RBD9103_InputGndString "RBD_INPUT_GND"

#define TIMEOUT 5

typedef enum RBD_UNT {
    RBD_UNT_NA = 0,
    RBD_UNT_UA = 1,
    RBD_UNT_MA = 2,
} RBDUnits_t;

typedef enum RBD_RANGE {
    RBD_RNG_AUTO = 0,
    RBD_RNG_2N = 1,
    RBD_RNG_20N = 2,
    RBD_RNG_200N = 3,
    RBD_RNG_2U = 4,
    RBD_RNG_20U = 5,
    RBD_RNG_200U = 6,
    RBD_RNG_2M = 7,
} RBDRange_t;

typedef enum RBD_ON_OFF {
    RBD_OFF = 0,
    RBD_ON = 1,
} RBDOnOff_t;

typedef enum RBD_FILTER_MODE {
    RBD_FLT_000 = 0,
    RBD_FLT_002 = 1,
    RBD_FLT_004 = 2,
    RBD_FLT_008 = 3,
    RBD_FLT_016 = 4,
    RBD_FLT_032 = 5,
    RBD_FLT_064 = 6,
} RBDFilterMode_t;

typedef enum RBD_SAMPLING_MODE {
    RBD_SAMPLING_MODE_SINGLE = 0,
    RBD_SAMPLING_MODE_MULTIPLE = 1,
    RBD_SAMPLING_MODE_CONTINUOUS = 2,
} RBDSamplingMode_t;

typedef enum RBD_RANGE_STATE {
    RBD_RANGE_STATE_OK = 0,
    RBD_RANGE_STATE_UNDER = 1,
    RBD_RANGE_STATE_OVER = 2,
} RBDRangeState_t;

using namespace std;

class RBD9103 : public asynPortDriver
{
public:
    RBD9103(const char* portName, const char* serialPortName);
    ~RBD9103();
    string writeReadCmd(const char* cmd);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus writeOctet(asynUser* pasynUser, const char* value, size_t nChars, size_t* nActual);
    void samplingThread();
protected:
private:
    asynUser* pasynUserSerialPort;
    bool alive;
    epicsThreadId samplingThreadId;
    void getDeviceStatus();
    void getModelNumber();
    void startSampling();
    void stopSampling();
    string splitRespOnDelim(string resp, const char* delim);
    void parseSampling(string rawSampling);
    void setSamplingRate(int rate);
    void appendCSV(double current, double avgCurrent, RBDRange_t range, RBDUnits_t currentUnits, int stable);
    RBDRange_t getRangeSettingFromStr(string rangeStr);
    const char* getStrFromRangeSetting(RBDRange_t range);
    RBDUnits_t getUnitsFromStr(string unitsStr);
    const char* getStrFromUnits(RBDUnits_t units);
    void openCSV(const char* filePath);
    void closeCSV();

    FILE* csvFile = NULL;

    int RBD9103_Model;
    int RBD9103_DrvVersion;
    int RBD9103_NumDigits;
    int RBD9103_SamplingRate;
    int RBD9103_SamplingRateActual;
    int RBD9103_Stable;
    int RBD9103_Sample;
    int RBD9103_State;
    int RBD9103_SamplingMode;
    int RBD9103_SampleCounter;
    int RBD9103_NumSamples;
    int RBD9103_CurrentUnits;
    int RBD9103_Current;
    int RBD9103_Range;
    int RBD9103_InRange;
    int RBD9103_OffsetNull;
    int RBD9103_RangeActual;
    int RBD9103_InputGnd;
    int RBD9103_Bias;
    int RBD9103_Filter;
    int RBD9103_AutoCal;
    int RBD9103_AutoGrounding;
    int RBD9103_DeviceID;
    int RBD9103_FwVersion;
    int RBD9103_BuildDate;
    int RBD9103_DeviceDesc;
    int RBD9103_AvgCurrent;
    int RBD9103_SumCurrent;
    int RBD9103_AvgCurrentUnits;
    int RBD9103_AvgOnlyStable;
    int RBD9103_Record;
    int RBD9103_FileName;
    int RBD9103_DirectoryPath;
    int RBD9103_DirExistsAndWritable;
    int RBD9103_Manufacturer;

    #define FIRST_RBD9103_PARAM RBD9103_Model
    #define LAST_RBD9103_PARAM RBD9103_Manufacturer

    #define NUM_RBD9103_PARAMS ((int)(&LAST_RBD9103_PARAM - &FIRST_RBD9103_PARAM + 1))

};

#endif
