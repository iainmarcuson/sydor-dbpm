/*
 * drvNSLS_EM.h
 * 
 * Asyn driver that inherits from the drvQuadEM class to control the NSLS Precision Integrator
 *
 * Author: Mark Rivers
 *
 * Created December 4, 2015
 */

#include "drvQuadEM.h"

#define MAX_COMMAND_LEN 256
#define MAX_MODULES 16
#define MAX_IPNAME_LEN 16
#define MAX_PORTNAME_LEN 32
#define MAX_RANGES 8

//PID Parameters
//X Channel
#define P_PIDXSpString              "PIDX_SP"                   /* asynFloat64,    r/w */
#define P_PIDXKPString              "PIDX_KP"                   /* asynFloat64,    r/w */
#define P_PIDXKIString              "PIDX_KI"                   /* asynFloat64,    r/w */
#define P_PIDXKDString              "PIDX_KD"                   /* asynFloat64,    r/w */
#define P_PIDXMVString              "PIDX_MAXV"                   /* asynFloat64,    r/w */
#define P_PIDXVOString              "PIDX_VOFF"                /* asynFloat64 */


//Y Channel
#define P_PIDYSpString              "PIDY_SP"                   /* asynFloat64,    r/w */
#define P_PIDYKPString              "PIDY_KP"                   /* asynFloat64,    r/w */
#define P_PIDYKIString              "PIDY_KI"                   /* asynFloat64,    r/w */
#define P_PIDYKDString              "PIDY_KD"                   /* asynFloat64,    r/w */
#define P_PIDYMVString              "PIDY_MAXV"                   /* asynFloat64,    r/w */
#define P_PIDYVOString              "PIDY_VOFF"                /* asynFloat64 */


//General PID
#define P_PIDEnableString           "PID_ON"                      /* asynInt32 */
#define P_PIDCutoffString           "PID_CUT"                     /* asynFloat64 */
#define P_PIDReenableString         "PID_RE"                      /* asynInt32 */
#define P_PIDDACModeString          "PID_DACM"                    /* asynInt32 */
#define P_PIDCutoutEnString         "PID_CUTEN"                   /* asynInt32 */
#define P_PIDCutoutHystString       "PID_CUTHY"                   /* asynFloat64 */
#define P_PIDIVString              "PID_ITOV"                    /* asynFloat64 */
#define P_PIDExtTrigString         "PID_XTRIG"                   /* asynInt32 */
#define P_PIDInhibitString         "PID_INHIBIT"                 /* asynInt32 */
#define P_PIDPosTrackString        "PID_POSTRACK"                /* asynInt32 */
#define P_PIDPosTrackRadString    "PID_TRACK_RAD"               /* asynFloat64 */

#define P_CalNameString   "CAL_NAME" /* asynOctet */
#define P_MaxCurrentString "I_MAX" /* asynFloat64 */

#define P_PIDRefreshString "PID_REFRESH"
#define P_RefreshAllString "REFRESH_ALL"

typedef struct {
    int moduleID;
    char moduleIP[MAX_IPNAME_LEN];
} moduleInfo_t;

typedef union {
  int out_int;
  double out_double;
} BS_Out_T;

typedef enum {
  reg_int,
  reg_double
} BS_Type_E;

typedef enum {
  param_reg,
  param_bit,
  param_multibit
} BS_Param_E;

typedef struct {
  BS_Param_E param_type;	/* Register, bit, or multibit */
  int reg_num;			/* Register address on B# */
  unsigned int bit_mask;	/* For single- and multi-bit parameters */
  BS_Type_E output_type; 	/* Int or float */
  double in_min;		/* Minimum value of input */
  double in_max;		/* Maximum value of input */
  BS_Out_T out_min;		/* Minimum of output, variable type */
  BS_Out_T out_max;		/* Maximum of output, variable type */
} Bs_Reg_T;

typedef struct {
  unsigned int cal_present;	/* Whether a calibration value is present */
  double cal_slope[4];		/* Calibration slope in nA */
  double cal_offset[4];		/* Calibration offset in nA */
} Calibration_Data;

typedef struct
{
  int lower_bound;		/* Set to -1 for not found */
  int upper_bound;		/* Set to -1 for not found */
} Cal_Find; 

/** Class to control the NSLS Precision Integrator */
class drvBS_EM : public drvQuadEM {
public:
    drvBS_EM(const char *portName, const char *broadcastAddress, int moduleID, int ringBufferSize);
    
    /* These are the methods we implement from asynPortDriver */
    void report(FILE *fp, int details);
                 
    /* These are the methods that are new to this class */
    void readThread(void);
    virtual void exitHandler();

    /* These are functions extended from drvQuadEM */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    
protected:
    /** Values used for pasynUser->reason, and indicies into the parameter library */
    int P_FdbkEnable;
#define FIRST_BS_COMMAND P_FdbkEnable
    int P_Fdbk_X_SP;
    int P_Fdbk_X_KP;
    int P_Fdbk_X_KI;
    int P_Fdbk_X_KD;
    int P_Fdbk_X_MaxV;
    int P_Fdbk_X_VOff;
    int P_Fdbk_Y_SP;
    int P_Fdbk_Y_KP;
    int P_Fdbk_Y_KI;
    int P_Fdbk_Y_KD;
    int P_Fdbk_Y_MaxV;
    int P_Fdbk_Y_VOff;
    int P_Fdbk_CutOutEn;
    int P_Fdbk_Reenable;
    int P_Fdbk_CutOutThresh;
    int P_Fdbk_CutOutHyst;
    int P_Fdbk_DACMode;
    int P_Fdbk_I2VScale;
    int P_Fdbk_ExtTrig;
    int P_Fdbk_PIDInhibit;
  int P_Fdbk_PosTrack;
  int P_Fdbk_PosTrackRad;

  int P_CalName;
  int P_MaxCurrent;

  int P_PIDRefresh;
  int P_RefreshAll;
  
    /* These are the methods we implement from quadEM */
    virtual asynStatus setAcquire(epicsInt32 value);
    virtual asynStatus setPingPong(epicsInt32 value);
    virtual asynStatus setIntegrationTime(epicsFloat64 value);
    virtual asynStatus setRange(epicsInt32 value);
    virtual asynStatus setValuesPerRead(epicsInt32 value);
    virtual asynStatus readStatus();
    virtual asynStatus reset();
 
private:
    /* Our data */
    char *broadcastAddress_;
    char udpPortName_[MAX_PORTNAME_LEN];
    char tcpCommandPortName_[MAX_PORTNAME_LEN];
    char tcpDataPortName_[MAX_PORTNAME_LEN];
    asynUser *pasynUserUDP_;
    asynUser *pasynUserTCPCommand_;
    asynUser *pasynUserTCPCommandConnect_;
    asynUser *pasynUserTCPData_;
    epicsEventId acquireStartEvent_;
    epicsEventId writeCmdEvent_;
    int moduleID_;
    int numModules_;
    double ranges_[MAX_RANGES];
    double scaleFactor_;
    moduleInfo_t moduleInfo_[MAX_MODULES];
    int readingActive_;
    char firmwareVersion_[MAX_COMMAND_LEN];
    char ipAddress_[MAX_IPNAME_LEN];
    char outString_[MAX_COMMAND_LEN];
    char inString_[MAX_COMMAND_LEN];
  char calName_[MAX_COMMAND_LEN];
    double acqFactor_;		/* Scaling factor for raw-to-current 
				   conversion from acquisition parameters */
    double adcFactor_;		/* Scaling factor from ADC parameters */
    double adcOffset_;		/* Offset from ADC parameters */
  double max_current_;		// XXX Not quite max current due to offset

    Bs_Reg_T pidRegData_[23];	/* Holds parameters for the PID registers */
    Calibration_Data cal_values_[MAX_RANGES]; /* One calibration per range */
    int num_cals_;			     /* Number of calibration values */
    double cal_slope_[4];			     /* The calculated slope */
    double cal_offset_[4];			     /* The calculated offset */

    asynStatus findModule();
    asynStatus writeReadMeter();
    asynStatus getFirmwareVersion();
    asynStatus setMode();
    asynStatus computeScaleFactor();
    void process_reg(int reg_lookup, double value);
    signed int current_to_raw(double current);
    double raw_to_current(signed int raw_val);
    void calc_calibration();
    void parse_cal_file(FILE *cal_file);
  asynStatus readResponse();
  void pvCallback(int *reg_pair);
};

