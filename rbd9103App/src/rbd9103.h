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


#define TIMEOUT 5

typedef enum RBD_UNT {
    RBD_UNT_NA = 0,
    RBD_UNT_UA = 1,
    RBD_UNT_MA = 2,
} RBDUnits_t;

class RBD9103 : public asynPortDriver
{
public:
    RBD9103(const char* portName, const char* serialPortName);
    ~RBD9103();
    asynStatus writeReadCmd(const char* cmd, char* ret, size_t maxChars, double timeout);
protected:
private:
    asynUser* pasynUserSerialPort;
    bool sampling;
    epicsThreadId samplingThread;

    int RBD9103_Model;
    int RBD9103_DrvVersion;
    int RBD9103_NumDigits;

    #define FIRST_RBD9103_PARAM RBD9103_Model
    #define LAST_RBD9103_PARAM RBD9103_NumDigits

    #define NUM_RBD9103_PARAMS ((int)(&LAST_RBD9103_PARAM - &FIRST_RBD9103_PARAM + 1))

};

#endif;
