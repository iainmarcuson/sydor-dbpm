/*
 * drvNSLS_EM.cpp
 * 
 * Asyn driver that inherits from the drvQuadEM class to control 
 * the NSLS Precision Integrator
 *
 * Author: Mark Rivers
 *
 * Created December 4, 2015
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsEvent.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <drvAsynIPPort.h>
#include <iocsh.h>

#include <epicsExport.h>

#include <arpa/inet.h>
#include <time.h>
#include "drvBS_EM.h"

#define BROADCAST_TIMEOUT 0.2
#define NSLS_EM_TIMEOUT   0.1

#define COMMAND_PORT    4747
#define DATA_PORT       5757
#define BROADCAST_PORT 37747
///XXX TODO Change according to hardware
///#define MIN_INTEGRATION_TIME 400e-6
#define MIN_INTEGRATION_TIME 880e-6
#define MAX_INTEGRATION_TIME 1.0
#define FREQUENCY 1e6
// 2^20 is maximum counts for 20-bit ADC
#define MAX_COUNTS ((0xFFFFF-0x01000)*1.0)
#define ADC_OFFSET 0x01000
typedef enum {
  Phase0, 
  Phase1, 
  PhaseBoth
} PingPongValue_t;

static const char *driverName="drvBS_EM";
static void readThread(void *drvPvt);


/** Constructor for the drvNSLS_EM class.
  * Calls the constructor for the drvQuadEM base class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] broadcastAddress The broadcast address of the network with this module
  * \param[in] moduleID The module ID of this module, set with rotary switch on module
  * \param[in] ringBufferSize The number of samples to hold in the input ring buffer.
  *            This should be large enough to hold all the samples between reads of the
  *            device, e.g. 1 ms SampleTime and 1 second read rate = 1000 samples.
  *            If 0 then default of 2048 is used.
  */
drvBS_EM::drvBS_EM(const char *portName, const char *broadcastAddress, int moduleID, int ringBufferSize) 
  : drvQuadEM(portName, ringBufferSize),
    pidRegData_{
  {param_reg, 200, 0xFFFFFFFF, reg_int, -1.0, 1.0, -10000, 10000}, //0 X Setpoint
    {param_reg, 201, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //1 X P term
      {param_reg, 202, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //2 X I term
	{param_reg, 203, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //3 X D term
	  {param_reg, 204, 0xFFFFFFFF, reg_int, 0.0, 5.0, 0, 50000},	 //4 X Max V
	    {param_reg, 205, 0xFFFFFFFF, reg_int, 0.0, 10.0, 0, 100000},	 //5 X Voltage Offset
	      {param_reg, 210, 0xFFFFFFFF, reg_int, -1.0, 1.0, -10000, 10000}, //6 Y Setpoint
		{param_reg, 211, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //7 Y P term
		  {param_reg, 212, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //8 Y I term
		    {param_reg, 213, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //9 Y D term
		      {param_reg, 214, 0xFFFFFFFF, reg_int, 0.0, 5.0, 0, 50000},	 //10 Y Max V
			{param_reg, 215, 0xFFFFFFFF, reg_int, 0.0, 10.0, 0, 100000},	 //11 Y Voltage Offset
	    {param_multibit, 240, 0x00000007, reg_int, 0, 0, 0 , 0}, //12 DAC mode
	      {param_bit, 220, 0x4, reg_int, 0, 0, 0, 0}, //13 PID enable
		{param_bit, 220, 0x2, reg_int, 0, 0, 0, 0}, //14 Cutout enable
		  {param_bit, 220, 0x1, reg_int, 0, 0, 0, 0}, //15 Auto reenable
		    {param_reg, 221, 0xFFFFFFFF, reg_int, 0, 1e6, 0, (int) 1e9}, //16 Cutout threshold
		      {param_reg, 222, 0xFFFFFFFF, reg_int, 0, 1e6, 0, (int) 1e9}, //17 Cutout reenable hysteresis
			{param_reg, 239, 0xFFFFFFFF, reg_int, 0, 1e5, 0, (int) 1e9}, //18 XXX FIXME I-to-V value?
			  {param_bit, 240, 0x8, reg_int, 0, 0, 0, 0}, //19 External trigger
			    {param_bit, 240, 0x10, reg_int, 0, 0, 0, 0} //20 PID Inhibit
}
  
{
    asynStatus status;
    const char *functionName = "drvBS_EM";
    char tempString[256];
    const char *cal_filename = "calibration.ini";
    FILE *cal_file;
    
    numModules_ = 0;
    moduleID_ = moduleID;
    ipAddress_[0] = 0;
    firmwareVersion_[0] = 0;
    // Integration capacitors in pF
    ranges_[0]=12;
    ranges_[1]=50;
    ranges_[2]=100;
    ranges_[3]=150;
    ranges_[4]=200;
    ranges_[5]=250;
    ranges_[6]=300;
    ranges_[7]=350;

    adcOffset_ = (double) 0x01000;
    adcFactor_ = 1.0/(0xFFFFF-adcOffset_);
    
    broadcastAddress_ = epicsStrDup(broadcastAddress);
    
    acquireStartEvent_ = epicsEventCreate(epicsEventEmpty);
    
    strcpy(udpPortName_, "UDP_");
    strcat(udpPortName_, portName);
    strcpy(tcpCommandPortName_, "TCP_Command_");
    strcat(tcpCommandPortName_, portName);
    strcpy(tcpDataPortName_, "TCP_Data_");
    strcat(tcpDataPortName_, portName);
    
    strcpy(tempString, broadcastAddress_);
    strcat(tempString, ":37747 UDP*");

    strcpy(tempString, broadcastAddress_);
    strcat(tempString, ":13001");
    //XXX Add error checking
    //Connect command port
    status = (asynStatus)drvAsynIPPortConfigure(tcpCommandPortName_, tempString, 0, 0, 0);
    if (status)
      {
	asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
		  "Error calling drvAsynIPPortConfigure for command port=%s, IP=%s, status=%d\n",
		  tcpCommandPortName_, tempString, status);
      }
    else
      {
	printf("Configured command port successfully.\n");
	fflush(stdout);
      }
    status = pasynOctetSyncIO->connect(tcpCommandPortName_, 0, &pasynUserTCPCommand_, NULL);
    if (status)
      {
	asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
		  "%s:%s: error calling drvAsynIPPortConfigure for Port: %s Command Address%s Status=%d\n",
		  driverName, functionName, tcpCommandPortName_, tempString, status);
      }
    else
      {
	printf("Connected command port Addr: %s.\n",tempString);
	fflush(stdout);
      }
    //status = pasynCommonSyncIO->connect(tcpCommandPortName_, 0, &pasynUserTCPCommandConnect_, NULL);
    epicsThreadSleep(1.0);
    printf("Completed sleep.\n");
    fflush(stdout);

    //Connect data port
    strcpy(tempString, broadcastAddress_);
    strcat(tempString, ":13002");
    printf("Data address is: %s\n", tempString);
    status = (asynStatus) drvAsynIPPortConfigure(tcpDataPortName_, tempString, 0, 0, 0);
    status = pasynOctetSyncIO->connect(tcpDataPortName_, 0, &pasynUserTCPData_, NULL);
    if (status)
      {
	printf("Error connecting data port.\n");
	fflush(stdout);
      }
    else
      {
	printf("Connected data port.\n");
	fflush(stdout);
      }

    cal_file = fopen(cal_filename, "r");
    if (cal_file == NULL)	// Assume there is no calibration file
      {
	int range_idx;

	printf("No calibration file detected; using defaults.\n");
	fflush(stdout);
	for (range_idx = 0; range_idx < MAX_RANGES; range_idx++)
	  {
	    int channel_idx;
	    cal_values_[range_idx].cal_present = 0; // No calibration
	    for (channel_idx = 0; channel_idx<4; channel_idx++)
	      {
		cal_values_[range_idx].cal_slope[channel_idx] = 1.0; // Make it a NOP just in case
		cal_values_[range_idx].cal_offset[channel_idx] = 0.0; // Ibid
	      }
	  }
	cal_values_[0].cal_present = 1; // Set to true; NOP set above
	num_cals_ = 1;			 // Flag one calibration value
      }
    else		      // A calibration file was found
      {
	int range_idx, channel_idx;
	parse_cal_file(cal_file);
	for (range_idx = 0; range_idx<MAX_RANGES; range_idx++)
	  {
	    printf("Calibration %02i, present %i\n", range_idx, cal_values_[range_idx].cal_present);
	    for (channel_idx = 0; channel_idx < 4; channel_idx++)
	      {
		printf("Channel %i Slope %8f Offset %8f\n", channel_idx, cal_values_[range_idx].cal_slope[channel_idx], cal_values_[range_idx].cal_offset[channel_idx]);
	      }
	  }
	fflush(stdout);
      }
    
    //Create the parameters for use with PID
    createParam(P_PIDEnableString, asynParamInt32, &P_FdbkEnable);
    createParam(P_PIDCutoffString, asynParamFloat64, &P_Fdbk_CutOutThresh);
    createParam(P_PIDReenableString, asynParamInt32, &P_Fdbk_Reenable);
    createParam(P_PIDCutoutEnString, asynParamInt32, &P_Fdbk_CutOutEn);
    createParam(P_PIDCutoutHystString, asynParamFloat64, &P_Fdbk_CutOutHyst);
    
    createParam(P_PIDXSpString, asynParamFloat64, &P_Fdbk_X_SP);
    createParam(P_PIDXKPString, asynParamFloat64, &P_Fdbk_X_KP);
    createParam(P_PIDXKIString, asynParamFloat64, &P_Fdbk_X_KI);
    createParam(P_PIDXKDString, asynParamFloat64, &P_Fdbk_X_KD);
    createParam(P_PIDXMVString, asynParamFloat64, &P_Fdbk_X_MaxV);
    createParam(P_PIDXVOString, asynParamFloat64, &P_Fdbk_X_VOff);
    
    createParam(P_PIDYSpString, asynParamFloat64, &P_Fdbk_Y_SP);
    createParam(P_PIDYKPString, asynParamFloat64, &P_Fdbk_Y_KP);
    createParam(P_PIDYKIString, asynParamFloat64, &P_Fdbk_Y_KI);
    createParam(P_PIDYKDString, asynParamFloat64, &P_Fdbk_Y_KD);
    createParam(P_PIDYMVString, asynParamFloat64, &P_Fdbk_Y_MaxV);
    createParam(P_PIDYVOString, asynParamFloat64, &P_Fdbk_Y_VOff);

    createParam(P_PIDDACModeString, asynParamInt32, &P_Fdbk_DACMode);
    createParam(P_PIDIVString, asynParamFloat64, &P_Fdbk_I2VScale);
    createParam(P_PIDExtTrigString, asynParamInt32, &P_Fdbk_ExtTrig);
    createParam(P_PIDInhibitString, asynParamInt32, &P_Fdbk_PIDInhibit);
    //Set the PID register parameters
    /*pidRegData_ = {
      {param_reg, 200, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //Setpoint
      {param_reg, 201, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //P term
      {param_reg, 202, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //I term
      {param_reg, 203, 0xFFFFFFFF, reg_int, 0.0, 1.0, 0, 10000}, //D term
      {param_reg, 204, 0xFFFFFFFF, reg_int, 0.0, 5.0, 0, 50000}	 //Max V
      }; */
       
    acquiring_ = 0;
    readingActive_ = 0;
    setIntegerParam(P_Model, QE_ModelNSLS_EM);
    
    // Do everything that needs to be done when connecting to the meter initially.
    // Note that the meter could be offline when the IOC starts, so we put this in
    // the reset() function which can be done later when the meter is online.
//    lock();
//    drvQuadEM::reset();
//    unlock();

    /* Create the thread that reads the meter */
    status = (asynStatus)(epicsThreadCreate("drvBS_EMTask",
                          epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::readThread,
                          this) == NULL);
    if (status) {
        printf("%s:%s: epicsThreadCreate failure, status=%d\n", driverName, functionName, status);
        return;
    }

    ///Parameters to set
    setIntegerParam(P_Range, 0);
    setIntegerParam(P_ValuesPerRead, 5);
    setDoubleParam(P_IntegrationTime, 10e-6);
    setDoubleParam(P_SampleTime, 20e-6);
    setIntegerParam(P_NumAverage, 25);


    callParamCallbacks();
}



static void readThread(void *drvPvt)
{
    drvBS_EM *pPvt = (drvBS_EM *)drvPvt;
    
    pPvt->readThread();
}

asynStatus drvBS_EM::findModule()
{
    size_t nwrite;
    size_t nread;
    epicsTimeStamp start;
    epicsTimeStamp now;
    epicsFloat64 deltaTime;
    int status;
    int eomReason;
    char buffer[1024];
    char *ptr;
    int count;
    char tempString[256];
    int i;
    static const char *functionName="findModules";

    status = pasynOctetSyncIO->write(pasynUserUDP_, "i\n", 2, 1.0, &nwrite);
    epicsTimeGetCurrent(&start);

    while (1) {
        epicsTimeGetCurrent(&now);
        deltaTime = epicsTimeDiffInSeconds(&now, &start);
        if (deltaTime > BROADCAST_TIMEOUT) break;
        status = pasynOctetSyncIO->read(pasynUserUDP_, buffer, sizeof(buffer), 0.01, &nread, &eomReason);
        if ((status == asynTimeout) && (nread > 0)) {
            ptr = buffer;
            while (1) {
                ptr = strstr(ptr, "id:");
                if (!ptr) break;
                sscanf(ptr, "id: %d%n", &moduleInfo_[numModules_].moduleID, &count);
                ptr += count;
                sscanf(ptr, " ip: %s%n", moduleInfo_[numModules_].moduleIP, &count);
                ptr += count;
                numModules_++;
            }
        } else if (status != asynTimeout) {
            return asynError;
        }
    }

    // See if the specified module was found
    for (i=0; i<numModules_; i++) {
        if (moduleInfo_[i].moduleID == moduleID_) break;
    }
    
    if (i == numModules_) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
            "%s:%s: cannot find requested module %d on network\n", 
            driverName, functionName, moduleID_);
        return asynError;
    }
    
    // Create TCP command port
    epicsSnprintf(tempString, sizeof(tempString), "%s:%d", moduleInfo_[i].moduleIP, COMMAND_PORT);
    // Set noAutoConnect, we will handle connection management
    status = drvAsynIPPortConfigure(tcpCommandPortName_, tempString, 0, 1, 0);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling drvAsyIPPortConfigure for TCP port %s, IP=%s, status=%d\n", 
            driverName, functionName, tcpCommandPortName_, moduleInfo_[i].moduleIP, status);
        return asynError;
    }

    // Connect to TCP command port
    status = pasynOctetSyncIO->connect(tcpCommandPortName_, 0, &pasynUserTCPCommand_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynOctetSyncIO->connect for TCP port, status=%d, error=%s\n", 
               driverName, functionName, status, pasynUserTCPCommand_->errorMessage);
        return asynError;
    }
    //pasynOctetSyncIO->setInputEos(pasynUserTCPCommand_, "\r\n", 2);
    //pasynOctetSyncIO->setOutputEos(pasynUserTCPCommand_, "\r", 1);
    status = pasynCommonSyncIO->connect(tcpCommandPortName_, 0, &pasynUserTCPCommandConnect_, NULL);
    if (status) {
        printf("%s:%s: error calling pasynCommonSyncIO->connect forTCP port, status=%d, error=%s\n", 
               driverName, functionName, status, pasynUserTCPCommand_->errorMessage);
        return asynError;
    }

    // Create TCP data port
    epicsSnprintf(tempString, sizeof(tempString), "%s:%d", moduleInfo_[i].moduleIP, DATA_PORT);
    status = drvAsynIPPortConfigure(tcpDataPortName_, tempString, 0, 0, 0);
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: error calling drvAsyIPPortConfigure for TCP port %s, IP=%s, status=%d\n", 
            driverName, functionName, tcpDataPortName_, moduleInfo_[i].moduleIP, status);
        return asynError;
    }

    // Connect to TCP data port
    status = pasynOctetSyncIO->connect(tcpDataPortName_, 0, &pasynUserTCPData_, NULL);
    if (status) {
        printf("%s:%s: error connecting to TCP port, status=%d, error=%s\n", 
               driverName, functionName, status, pasynUserTCPData_->errorMessage);
        return asynError;
    }
    pasynOctetSyncIO->setInputEos(pasynUserTCPData_, "\n", 1);

    return asynSuccess;
}

void drvBS_EM::process_reg(int reg_lookup, double value)
{
  double t;
  Bs_Reg_T curr_item;
  BS_Out_T out_value;
  
  curr_item = pidRegData_[reg_lookup];

  if (curr_item.param_type == param_multibit)
    {
      int reg_start_value;
      char *delim_find;
      char response_string[256];
      
      //First, read in the value
      epicsSnprintf(outString_, sizeof(outString_), "rr %d?\r\n", curr_item.reg_num);
      writeReadMeter();
      sscanf(inString_, "%[^\n]", response_string);
      printf("Multi-bit response string reg %d, length %i:\n%s\n", curr_item.reg_num, (int) strlen(response_string), response_string);
      fflush(stdout);
      
      delim_find = strstr(inString_, ">");
      if (delim_find == NULL)	// TODO Handle this better
	{
	  epicsSnprintf(outString_, sizeof(outString_), "rr 1?\r\n"); // Put in a harmless command
	  printf("Failed to get response from hardware.\n");
	  fflush(stdout);
	  return;
	}
      
      sscanf(&(delim_find[1]), "%i", &reg_start_value); // Start after the delimiter
      reg_start_value = reg_start_value & ~curr_item.bit_mask; // Mask out the bits to manipulate
      reg_start_value = reg_start_value | (unsigned int) value; // Or in the new value from the Db file
      epicsSnprintf(outString_, sizeof(outString_), "wr %d %i\r\n", curr_item.reg_num, reg_start_value);
      return;
    }

  if (curr_item.param_type == param_bit)
    {
      unsigned int bitmask;

      bitmask = curr_item.bit_mask;
      if (value)
	{
	  epicsSnprintf(outString_, sizeof(outString_), "bs %i %u\r\n", curr_item.reg_num, bitmask);
	  return;
	}
      else
	{
	  epicsSnprintf(outString_, sizeof(outString_), "bc %i %u\r\n", curr_item.reg_num, bitmask);
	  return;
	}
    }
  
  t = (value-curr_item.in_min)/(curr_item.in_max-curr_item.in_min);
  if (curr_item.output_type == reg_int)
    {
      int out_val;
      out_val = curr_item.out_min.out_int + t*(curr_item.out_max.out_int-curr_item.out_min.out_int);
      epicsSnprintf(outString_, sizeof(outString_), "wr %d %d\r\n", curr_item.reg_num, out_val);
    }
  else
    {
      printf("Error: Unimplemented function for write float.\n");
      fflush(stdout);
    }
  
  return;
}
/** Writes a string to the BS_EM and reads the response. */
asynStatus drvBS_EM::writeReadMeter()
{
  size_t nread;
  size_t nwrite;
  asynStatus status=asynSuccess;
//  char tempString[16];
  int eomReason;
  static const char *functionName="writeReadMeter";

  ///XXX Debugging
  printf("Starting writeReadMeter.\n");
  fflush(stdout);

  // The meter has a strange behavior.  Commands that take no arguments succeed on the first write/read
  // but commands that take arguments fail on the first write read, must do it again.
  ///XXX
  ///pasynCommonSyncIO->connectDevice(pasynUserTCPCommandConnect_);
  
  status = pasynOctetSyncIO->writeRead(pasynUserTCPCommand_, outString_, strlen(outString_), 
                                          inString_, sizeof(inString_), NSLS_EM_TIMEOUT, 
                                          &nwrite, &nread, &eomReason);
  if (status) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
          "%s:%s: error calling writeRead, outString=%s status=%d, nread=%d, eomReason=%d, inString=%s\n",
          driverName, functionName, outString_, status, (int)nread, eomReason, inString_);
  }
  else if (strstr(inString_, ">:OK") == 0) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
          "%s:%s: error, outString=%s expected >:OK, received %s\n",
          driverName, functionName, outString_, inString_);
      status = asynError;
  }
  ///XXX
  ///pasynCommonSyncIO->disconnectDevice(pasynUserTCPCommandConnect_);
  
  printf("Finishhhhed             writeReadMer.\n");;;;;;;
  fflush(stdout);

  return status;
}

/** Read thread to read the data from the electrometer when it is in continuous acquire mode.
  * Reads the data, computes the sums and positions, and does callbacks.
  */

void drvBS_EM::readThread(void)
{
    asynStatus status;
    size_t nRead;
    size_t total_read;
    int eomReason;
    int pingPong;
    int i,j,k;
    asynUser *pasynUser;
    asynInterface *pasynInterface;
    asynOctet *pasynOctet;
    void *octetPvt;
    int phase;
    epicsInt32 raw[4];
    epicsFloat64 data[4];
    static char ASCIIData[4096];
    size_t nRequested;
    int num_data;		// Number of current words
    int num_words;		// Number of received words 
    unsigned int *data_int = (unsigned int *)ASCIIData;
    static const char *functionName = "readThread";
    int total_num = 0;
    ///Benchmarking
#define RATE_BENCHMARK
    ///#undef RATE_BENCHMARK
#ifdef RATE_BENCHMARK
    const char *rate_log_filename = "ratelog.txt";
    FILE *rate_log_file;
    time_t old_minutes, curr_minutes, curr_time;
    int total_bytes, packet_err_count;
    int recv_count;
#endif

#ifdef RATE_BENCHMARK
    total_bytes = 0;
    packet_err_count = 0;
    curr_time = time(NULL);
    // old_minutes = curr_time/60;
    curr_minutes = curr_time/60;
    rate_log_file = fopen(rate_log_filename, "a");
    recv_count = 0;
    printf("Invoked readThread()\n");
#endif

    /* Create an asynUser */
    pasynUser = pasynManager->createAsynUser(0, 0);
    pasynUser->timeout = NSLS_EM_TIMEOUT;
    status = pasynManager->connectDevice(pasynUser, tcpDataPortName_, 0);
    if(status!=asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: connectDevice failed, status=%d\n",
            driverName, functionName, status);
    }
    pasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);
    if(!pasynInterface) {;
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: findInterface failed for asynOctet, status=%d\n",
            driverName, functionName, status);
    }
    pasynOctet = (asynOctet *)pasynInterface->pinterface;
    octetPvt = pasynInterface->drvPvt;
    
    /* Loop forever */
    lock();
    while (1) {
        if (acquiring_ == 0) {
            readingActive_ = 0;
            unlock();
            (void)epicsEventWait(acquireStartEvent_);
            lock();
            readingActive_ = 1;
            status = pasynOctet->flush(octetPvt, pasynUser);
            getIntegerParam(P_PingPong,       &pingPong);
            nRequested = sizeof(ASCIIData);
        }
        unlock();
        pasynManager->lockPort(pasynUser);
//Read header
	status = pasynOctet->read(octetPvt, pasynUser, ASCIIData, 4, &nRead, &eomReason);
	nRequested = ntohs(*(unsigned short *)(&ASCIIData[2]));
	num_words = (nRequested-1)/4; // Subtract 1-byte checksum
	num_data = (nRequested-13)/4; //12-byte header plus 1-byte checksum
#ifdef RATE_BENCHMARK
	total_bytes += 4;
#endif

	total_read = 0;
	///printf("readThread: NumRead=%d, first byte=%hhx, NumBytes=%d.\n",
	///       nRead, ASCIIData[0], nRequested);
	fflush(stdout);
	//Check for valid data
	if ((ASCIIData[0] != 'B') || (ASCIIData[1] != 1))
	  {
#ifdef RATE_BENCHMARK
            packet_err_count++;
	    if ((packet_err_count % 30) == 0)
	      {
		///printf("%hhu, %hhu, %hhu, %hhu\n", ASCIIData[0], ASCIIData[1], ASCIIData[2], ASCIIData[3]);
	      }
#endif RATE_BENCHMARK
	    pasynOctet->flush(octetPvt, pasynUser);
	    pasynManager->unlockPort(pasynUser);
	    lock();
	    continue;		// Try again next packet
	  }
	///XXX TODO Note that the checksum is not currently transmitted, so the +1 that should be there has been turne into a +0 for now
	while(total_read < (nRequested+0))
	  {
	    status = pasynOctet->read(octetPvt, pasynUser, &(ASCIIData[total_read]), nRequested+0-total_read, &nRead, &eomReason);
	    total_read += nRead;
	  }

#ifdef RATE_BENCHMARK
	recv_count++;
	total_bytes += nRead;
	curr_time = time(NULL);
	curr_minutes = curr_time/60;
	if ((recv_count % 100) == 0)
	  {
	    //printf("Requested %lu bytes, received %lu bytes.\n", nRequested+0, total_read);
	    //printf("Final byte is %hhx\n", ASCIIData[total_read-1]);
	  }
  //printf("Recevied 100 packets, time %i, current %i, old %i\n", curr_time, curr_minutes, old_minutes);
  //fflush(stdout);

  


	if (curr_minutes != old_minutes)
	  {
	    //printf("Found a difference.\n");
	    //fflush(stdout);
	    old_minutes = curr_minutes;
	    fprintf(rate_log_file, "%i\t%i\t%i %e\n", packet_err_count, recv_count, packet_err_count+recv_count, ((double)total_bytes)/60.0);
	    if ((curr_minutes % 3) == 0)
	      {
		fflush(rate_log_file);
	      }
	    total_bytes = 0;
	    packet_err_count = 0;
	    recv_count = 0;
	  }
#endif

	
        ///printf("readThread: Read %i data bytes.\n", nRead);
	fflush(stdout);
	///pasynOctet->flush(octetPvt, pasynUser);
        pasynManager->unlockPort(pasynUser);
        lock();

	if (0){
        if ((status != asynSuccess) || 
            (eomReason != ASYN_EOM_EOS)) {
            if (status != asynTimeout) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, 
                    "%s:%s: unexpected error reading meter status=%d, nRead=%lu, eomReason=%d\n", 
                    driverName, functionName, status, (unsigned long)nRead, eomReason);
                // We got an error reading the meter, it is probably offline.  
                // Wait 1 second before trying again.
                unlock();
                epicsThreadSleep(1.0);
                lock();
            }
            continue;
        }
	}
	/// Make the data little endian
	///printf("Making data little endian.\n");
	///fflush(stdout);
	for (i=0; i<num_words; i++)
	  {
	    data_int[i] = ntohl(data_int[i]);
	    ///data_int[i] = total_num++;
	  }

	for (j=0; j<num_data/4; j++)
	  {
	    if (((phase == 0) && (pingPong == Phase0)) ||
		((phase == 1) && (pingPong == Phase1)) ||
		(pingPong == PhaseBoth)) {
	      for (i=0; i<4; i++) {
		//12 bytes offset of payload, so 3 ints
		data[i] = raw_to_current(data_int[j*4+i+3]);
		///TODO Add in calibration
		data[i] = data[i] - (cal_offset_[i]*1e-9);
		data[i] = data[i]/cal_slope_[i];
	      }      
	      ///
	      ///printf("Computing positions.\n");
	      fflush(stdout);
	      computePositions(data);
	    }
	  }
    }
}

/** Functions to handle writing of parameters */
asynStatus drvBS_EM::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  int channel;
  int reg_lookup;
  const char *paramName;
  const char *functionName = "writeInt32";

  getAddress(pasynUser, &channel);

  /* Set the parameter in the parameter library. */
  status |= setIntegerParam(channel, function, value);

  // Fetch the parameter string name
  getParamName(function, &paramName);

  printf("In writeInt32()\n");

  reg_lookup = -1;
  if (function == P_FdbkEnable)
    {
      reg_lookup = 13;
    }
  else if (function == P_Fdbk_DACMode)
    {
      reg_lookup = 12;
    }
  else if (function == P_Fdbk_Reenable)
    {
      reg_lookup = 15;
    }
  else if (function == P_Fdbk_CutOutEn)
    {
      reg_lookup = 14;
    }
  else if (function == P_Fdbk_ExtTrig)
    {
      reg_lookup = 19;
    }
  else if (function == P_Fdbk_PIDInhibit)
    {
      reg_lookup = 20;
    }
  
  if (reg_lookup >= 0)
    {
      process_reg(reg_lookup, value);	// Get the command string for the register lookup set
      status = writeReadMeter();
    }

  if (function < P_FdbkEnable)	// Assume function not a BSharp one
    {
      printf("writeINt falling through.\n");
      if (function == P_Acquire)
	{
	  printf("writing an acquire in fallthrough\n");
	}
      fflush(stdout);
      drvQuadEM::writeInt32(pasynUser, value);
    }

  // Do callbacker for higher layers
  status = (asynStatus) callParamCallbacks();

  //TODO FIXME Handle status codes
  return (asynStatus)status;
}

// TODO Expand this
asynStatus drvBS_EM::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  int status = asynSuccess;
  int channel;
  int reg_lookup;

  ///XXX REMOVETHIS
  //return drvQuadEM::writeFloat64(pasynUser, value);
  
  getAddress(pasynUser, &channel);

  // Set the parameter in the parameter library.
  status |= setDoubleParam(channel, function, value);
  
  // Look up the specific function
  reg_lookup = -1;
  
  if (function == P_Fdbk_X_SP)
    {
      reg_lookup = 0;
    }
  else if (function == P_Fdbk_X_KP)
    {
      reg_lookup = 1;
    }
  else if (function == P_Fdbk_X_KI)
    {
      reg_lookup = 2;
    }
  else if (function == P_Fdbk_X_KD)
    {
      reg_lookup = 3;
    }
  else if (function == P_Fdbk_X_MaxV)
    {
      reg_lookup = 4;
    }
  else if (function == P_Fdbk_X_VOff)
    {
      reg_lookup = 5;
    }
  else if (function == P_Fdbk_Y_SP)
    {
      reg_lookup = 6;
    }
  else if (function == P_Fdbk_Y_KP)
    {
      reg_lookup = 7;
    }
  else if (function == P_Fdbk_Y_KI)
    {
      reg_lookup = 8;
    }
  else if (function == P_Fdbk_Y_KD)
    {
      reg_lookup = 9;
    }
  else if (function == P_Fdbk_Y_MaxV)
    {
      reg_lookup = 10;
    }
  else if (function == P_Fdbk_Y_VOff)
    {
      reg_lookup = 11;
    }
  else if (function == P_Fdbk_CutOutThresh)
    {
      reg_lookup = 16;
    }
  else if (function == P_Fdbk_CutOutHyst)
    {
      reg_lookup = 17;
    }
  else if (function == P_Fdbk_I2VScale)
    {
      reg_lookup = 18;
    }
	
    
  if (reg_lookup >= 0)
    {
      process_reg(reg_lookup, value);	// Get the command string for the register lookup set
      printf("%s\n", outString_);
      fflush(stdout);
      status = writeReadMeter();
    }

  return (asynStatus)drvQuadEM::writeFloat64(pasynUser, value);
}

					     
/** Starts and stops the electrometer.
  * \param[in] value 1 to start the electrometer, 0 to stop it.
  */
asynStatus drvBS_EM::setAcquire(epicsInt32 value) 
{
    //static const char *functionName = "setAcquire";

    // Return without doing anything if value=1 and already acquiring
    if ((value == 1) && (acquiring_)) return asynSuccess;
    
    if (value == 0) {
        // Setting this flag tells the read thread to stop
        acquiring_ = 0;
        // Wait for the read thread to stop
        while (readingActive_) {
            unlock();
            epicsThreadSleep(0.01);
            lock();
        }
    } else {
        // Notify the read thread if acquisition status has started
        epicsEventSignal(acquireStartEvent_);
        acquiring_ = 1;
    }
    // Call the base class function in case anything needs to be done there.
    drvQuadEM::setAcquire(value);
    return setMode();
}

/** Set the acquisition mode
  */
asynStatus drvBS_EM::setMode()
{
    int pingPong;
    int valuesPerRead;
    int acquire;
    int mode;

    getIntegerParam(P_PingPong, &pingPong);
    getIntegerParam(P_ValuesPerRead, &valuesPerRead);
    getIntegerParam(P_Acquire, &acquire);
    mode = P_Acquire ? 0 : 1;
    // The phase information is only valid when ValuesPerRead=1.  Set to PhaseBoth if !=1.
    if ((valuesPerRead != 1) && (pingPong != PhaseBoth)) {
        pingPong = PhaseBoth;
        setIntegerParam(P_PingPong, PhaseBoth);
    }
    if (pingPong != PhaseBoth) mode |= 0x80;
    // Send the mode command
    sprintf(outString_, "m %d", mode);
    ///writeReadMeter();
    return asynSuccess;
}

/** Sets the integration time. 
  * \param[in] value The integration time in seconds [0.001 - 1.000]
  */
asynStatus drvBS_EM::setIntegrationTime(epicsFloat64 value) 
{
    asynStatus status;
    int time_scale_num;	// Number to send that corresponds to time
    /* Make sure the integration time is valid. If not change it and put back in parameter library */
    if (value < MIN_INTEGRATION_TIME) {
        value = MIN_INTEGRATION_TIME;
        setDoubleParam(P_IntegrationTime, value);
    }
    ///XXX TODO Check return code
    time_scale_num = (value-MIN_INTEGRATION_TIME)/(66.6e-9);
    epicsSnprintf(outString_, sizeof(outString_), "wr 1 %d\r\n", time_scale_num);
    status = writeReadMeter();
    epicsSnprintf(outString_, sizeof(outString_), "wr 2 %d\r\n", time_scale_num);
    status = writeReadMeter();
    computeScaleFactor();
    return status;
}

/** Sets the range 
  * \param[in] value The desired range.
  */
asynStatus drvBS_EM::setRange(epicsInt32 value) 
{
    asynStatus status;
    
    epicsSnprintf(outString_, sizeof(outString_), "wr 3 %d\r\n", value);
    status = writeReadMeter();
    computeScaleFactor();
    return status;
}


/** Sets the values per read.
  * \param[in] value Values per read. Minimum depends on number of channels.
  */
asynStatus drvBS_EM::setValuesPerRead(epicsInt32 value) 
{
    asynStatus status;
    
    epicsSnprintf(outString_, sizeof(outString_), "n %d", value);
    //status = writeReadMeter();
    computeScaleFactor();
    return setMode();
}

 /** Sets the ping-pong setting. 
  * \param[in] value 0: use both ping and pong (HLF OFF), value=1: use just ping (HLF ON) 
  */
asynStatus drvBS_EM::setPingPong(epicsInt32 value) 
{
    return setMode();
}

asynStatus drvBS_EM::computeScaleFactor()
{
    int range;
    int valuesPerRead;
    double integrationTime;
    static const char *functionName = "computeScaleFactor";

    getIntegerParam(P_ValuesPerRead,  &valuesPerRead);
    getIntegerParam(P_Range,          &range);
    getDoubleParam(P_IntegrationTime, &integrationTime);
    scaleFactor_ = ranges_[range]*1e-12 * FREQUENCY / (integrationTime * 1e6)
                  / MAX_COUNTS / (double)valuesPerRead;
    acqFactor_ = ranges_[range]*1e-12*FREQUENCY/(integrationTime*1e6)/(double) valuesPerRead;
    this->calc_calibration();
    ///
    if (1)
      {
	int channel_idx;
	printf("New calibration values:\n");
	for (channel_idx =0; channel_idx < 4; channel_idx++)
	  {
	    printf("Channel %i Slope: %8f Offset: %8f\n", channel_idx, cal_slope_[channel_idx], cal_offset_[channel_idx]);
	  }
	fflush(stdout);
      }

    while (1)			// XXX Always do this, but break at end
      {
	int channel_idx;

	for (channel_idx = 0; channel_idx<4; channel_idx++)
	  {
	    epicsSnprintf(outString_, sizeof(outString_), "wr %i %i\r\n", 230+channel_idx, (int) (cal_slope_[channel_idx]*10000));
	    writeReadMeter();	// XXX Doesn't test return value
	    epicsSnprintf(outString_, sizeof(outString_), "wr %i %i\r\n", 234+channel_idx, (int) (cal_offset_[channel_idx]*10000));
	    writeReadMeter();
	  }
	break;			// Leave this loop -- one time only
      }
    
    ///XXX FIXME TODO Set here to allow setIntegrationTime to shift current
    //scaleFactor_ = ranges_[range]*1e-12 * FREQUENCY / (1.0 * 1e6)
    //              / MAX_COUNTS / (double)valuesPerRead;
    asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
        "%s::%s scaleFactor=%e\n", driverName, functionName, scaleFactor_);
    return asynSuccess;
}


/** Reads all the settings back from the electrometer.
  */
asynStatus drvBS_EM::readStatus() 
{
    // Reads the values of all the meter parameters, sets them in the parameter library
    int mode, valuesPerRead, range;
    double period;
    int numAverage;
    int pingPong;
    int numItems;
    double sampleTime, averagingTime;
    static const char *functionName = "getStatus";
    
    ///Fill in with either resonable values or the parameter values
    
    getIntegerParam(P_Range, &range);
    setIntegerParam(P_Range, range);
    valuesPerRead = 1;
    setIntegerParam(P_ValuesPerRead, valuesPerRead);
    getDoubleParam(P_IntegrationTime, &period);
    ///
    printf("Integration time %f.\n", period);
    fflush(stdout);
    setDoubleParam(P_IntegrationTime, period);
    sampleTime = period*valuesPerRead;
    getIntegerParam(P_PingPong, &pingPong);
    if (pingPong != PhaseBoth) sampleTime *= 2.0;
    setDoubleParam(P_SampleTime, sampleTime);
    
    // Compute the number of values that will be accumulated in the ring buffer before averaging
    getDoubleParam(P_AveragingTime, &averagingTime);
    numAverage = (int)((averagingTime / sampleTime) + 0.5);
    setIntegerParam(P_NumAverage, numAverage);
    ///
    printf("Number averaging: %i.\n", numAverage);
    return asynSuccess;
}

asynStatus drvBS_EM::reset()
{
    asynStatus status;
    //static const char *functionName = "reset";

    // Call the base class method
    status = drvQuadEM::reset();
    return status;
}


/** Exit handler.  Turns off acquire so we don't waste network bandwidth when the IOC stops */
void drvBS_EM::exitHandler()
{
    lock();
    setAcquire(0);
    unlock();
}

signed int drvBS_EM::current_to_raw(double current)
{
  signed int raw_val;
  raw_val = current/(adcFactor_*acqFactor_)+adcOffset_;
  ///TODO FIXME Put in clipping
  return raw_val;
}

double drvBS_EM::raw_to_current(signed int raw_val)
{
  double current;
  current = (raw_val-adcOffset_)*adcFactor_*acqFactor_;
  return current;
}

void drvBS_EM::parse_cal_file(FILE *cal_file)
{
  char curr_line[256];
  int range_idx;
  int channel_idx;

  // Note that there are zero calibration values so far
  num_cals_ = 0;
  
  // First, initialize the array to NOPs
  for (range_idx = 0; range_idx < MAX_RANGES; range_idx++)
    {
      cal_values_[range_idx].cal_present = 0; // Set to absent
      for (channel_idx = 0; channel_idx < 4; channel_idx++)
	{
	  cal_values_[range_idx].cal_slope[channel_idx] = 1.0;
	  cal_values_[range_idx].cal_offset[channel_idx] = 0.0;
	}
    }
  
  while (!feof(cal_file))
    {
      char curr_channel;	// Channels are letters
      int channel_num;		// Numeric channel index
      int curr_range;		// Range being set
      double curr_slope;	// Slope for this value
      double curr_offset;	// Offset for this value
      int args_read;
      int upper_letter;		// Uppercase channel letter
      
      ///FIXME handle long lines appropriately
      fscanf(cal_file, "%[^\n]%*[\n]", curr_line);
    found_cal_line:		// A potential calibration line from later
      args_read = sscanf(curr_line, " [direct_range%i] ", &curr_range);

      if (args_read == 0)	// Range not found
	{
	  continue;		// Try with next line
	}


      if ((curr_range < 0) || (curr_range >= MAX_RANGES)) // Out of range
	{
	  continue;		// Try again with next line
	}

      cal_values_[curr_range].cal_present = 1; // Flag calibration present
      num_cals_++;			       // Increase calibration count
      
      while (!feof(cal_file))			// Loop forever to read in new lines to get
	// calibration values
	{
	  fscanf(cal_file, "%[^\n]%*[\n]", curr_line); // Read in a new line
	  args_read = sscanf(curr_line, " %1[[]", &curr_channel); // Check for new range line
	  if (args_read == 1)					 // Found a new range line
	    {
	      goto found_cal_line; // Start processing from there
	    }
	  args_read = sscanf(curr_line, " Channel%c = \" %lf , %lf \"", &curr_channel, &curr_slope, &curr_offset);
	  if (args_read != 3)	// Not a valid calibration line
	    {
	      continue;		// Try again
	    }

	  upper_letter = toupper(curr_channel);
	  channel_num = upper_letter - 'A'; // A-D
	  if ((channel_num < 0) || (channel_num >= 4)) // Out of range
	    {
	      continue;		// Try again
	    }

	  // Populate values
	  cal_values_[curr_range].cal_slope[channel_num] = curr_slope;
	  cal_values_[curr_range].cal_offset[channel_num] = curr_offset;
	}
    }

  return;
}

void drvBS_EM::calc_calibration()
{
  int lower_bound = -1;		// Not found yet
  int upper_bound = -1;		// Not found yet
  double curr_slope;
  double curr_offset;		// Current slope and offset
  double range_low;		// Lower range charge value for interpolation
  double range_high;		// Higher range charge value for interpolation
  double range_set;		// Curent range charge value
  
  int curr_range;		// Value of current range
  int range_idx, channel_idx;	// Indices for iterating
  if (num_cals_ == 0)
    {
      int channel_idx;
      for (channel_idx = 0; channel_idx <  4; channel_idx++)
	{
	  cal_slope_[channel_idx] = 1.0;
	  cal_offset_[channel_idx] = 0.0;	// Set for NOPs
	}
      return;
    }
  else if (num_cals_ == 1)	// One calibration value
    {
      int range_idx;
            
      for (range_idx = 0; range_idx<MAX_RANGES; range_idx++)
	{
	  if (cal_values_[range_idx].cal_present == 1) // Found THE present value
	    {
	      int channel_idx;

	      for (channel_idx = 0; channel_idx < 4; channel_idx++)
		{
		  cal_slope_[channel_idx] = cal_values_[range_idx].cal_slope[channel_idx];
		  cal_offset_[channel_idx] = cal_values_[range_idx].cal_offset[channel_idx];
		}
	      break;
	    }
	}
      return;
    }

  // Now there are multiple calibration values set
  // Iterate upwards

  getIntegerParam(P_Range, &curr_range);
  for (range_idx = 0; range_idx<MAX_RANGES; range_idx++)
    {
      if (cal_values_[range_idx].cal_present == 1) // We have found a calibration
	{
	  if (range_idx < curr_range) // Range lower
	    {
	      lower_bound = range_idx; // Note a new lower value
	    }
	  else if (range_idx == curr_range) // Exact match
	    {
	      for (channel_idx = 0; channel_idx<4; channel_idx++)
		{
		  cal_slope_[channel_idx] = cal_values_[range_idx].cal_slope[channel_idx];
		  cal_offset_[channel_idx] = cal_values_[range_idx].cal_offset[channel_idx];
		}
	      return;
	    }
	  else 			// Range greater than index
	    {
	      upper_bound = range_idx;
	      break;		// No need to check more
	    }
	}
    } // for range_idx=...

  if ((lower_bound >= 0) && (upper_bound < 0)) //only lower values found
    {
      for (channel_idx = 0; channel_idx < 4; channel_idx++)
	{
	  cal_slope_[channel_idx] = cal_values_[lower_bound].cal_slope[channel_idx];
	  cal_offset_[channel_idx] = cal_values_[lower_bound].cal_offset[channel_idx];
	}
    }
  else if ((lower_bound < 0) && (upper_bound >= 0)) //only upper values found
    {
      for (channel_idx = 0; channel_idx < 4; channel_idx++)
	{
	  cal_slope_[channel_idx] = cal_values_[upper_bound].cal_slope[channel_idx];
	  cal_offset_[channel_idx] = cal_values_[upper_bound].cal_offset[channel_idx];
	}
    }
  else				// Bounded on both sides
    {
      range_low = ranges_[lower_bound];
      range_high = ranges_[upper_bound];
      range_set = ranges_[curr_range];
      
      for (channel_idx = 0; channel_idx<4; channel_idx++)
	{
	  curr_slope = cal_values_[lower_bound].cal_slope[channel_idx]*(range_high-range_set) + cal_values_[upper_bound].cal_slope[channel_idx]*(range_set-range_low);
	  curr_slope = curr_slope/(range_high-range_low);
	  curr_offset = cal_values_[lower_bound].cal_offset[channel_idx]*(range_high-range_set) + cal_values_[upper_bound].cal_offset[channel_idx]*(range_set-range_low);
	  curr_offset = curr_offset/(range_high-range_low);
	  cal_slope_[channel_idx] = curr_slope;
	  cal_offset_[channel_idx] = curr_offset;
	} // for channel_idx
      return;
    }

  return;
}
  

/** Report  parameters 
  * \param[in] fp The file pointer to write to
  * \param[in] details The level of detail requested
  */
void drvBS_EM::report(FILE *fp, int details)
{
    int i;
    
    fprintf(fp, "%s: port=%s\n", driverName, portName);
    if (details > 0) {
        fprintf(fp, "  IP address:       %s\n", ipAddress_);
        fprintf(fp, "  Module ID:        %d\n", moduleID_);
        fprintf(fp, "  Firmware version: %s\n", firmwareVersion_);
        fprintf(fp, "  Scale factor:     %e\n", scaleFactor_);
    }
    fprintf(fp, "  Number of modules found on network=%d\n", numModules_);
    for (i=0; i<numModules_; i++) {
        fprintf(fp, "    Module %d\n", i);
        fprintf(fp, "      Module ID:  %d\n", moduleInfo_[i].moduleID);
        fprintf(fp, "      IP address: %s\n", moduleInfo_[i].moduleIP);
    }
    drvQuadEM::report(fp, details);
}


/* Configuration routine.  Called directly, or from the iocsh function below */

extern "C" {

/** EPICS iocsh callable function to call constructor for the drvNSLS_EM class.
  * \param[in] portName The name of the asyn port driver to be created.
  * \param[in] broadcastAddress The broadcast address of the network with this module
  * \param[in] moduleID The module ID of this module, set with rotary switch on module
  * \param[in] ringBufferSize The number of samples to hold in the input ring buffer.
  *            This should be large enough to hold all the samples between reads of the
  *            device, e.g. 1 ms SampleTime and 1 second read rate = 1000 samples.
  *            If 0 then default of 2048 is used.
  */
int drvBS_EMConfigure(const char *portName, const char *broadcastAddress, int moduleID, int ringBufferSize)
{
    new drvBS_EM(portName, broadcastAddress, moduleID, ringBufferSize);
    return(asynSuccess);
}


/* EPICS iocsh shell commands */

static const iocshArg initArg0 = { "portName", iocshArgString};
static const iocshArg initArg1 = { "broadcast address", iocshArgString};
static const iocshArg initArg2 = { "module ID", iocshArgInt};
static const iocshArg initArg3 = { "ring buffer size",iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0,
                                            &initArg1,
                                            &initArg2,
                                            &initArg3};
static const iocshFuncDef initFuncDef = {"drvBS_EMConfigure",4,initArgs};
static void initCallFunc(const iocshArgBuf *args)
{
    drvBS_EMConfigure(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void drvBS_EMRegister(void)
{
    iocshRegister(&initFuncDef,initCallFunc);
}

epicsExportRegistrar(drvBS_EMRegister);

}

