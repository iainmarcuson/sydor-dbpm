/* EPICS includes */
#include <epicsTypes.h>

typedef struct {
	epicsUInt16 mask;
	epicsUInt16 wentLow, wentHigh;
	void *userPvt;
} softGlueIntRoutineData;

int softGlueRegisterInterruptRoutine(epicsUInt16 carrier, epicsUInt16 slot, int sopcAddress, epicsUInt16 mask,
	void (*routine)(softGlueIntRoutineData *IRData), void *userPvt);
epicsUInt16 *softGlueCalcSpecifiedRegisterAddress(epicsUInt16 carrier, epicsUInt16 slot, int addr);
