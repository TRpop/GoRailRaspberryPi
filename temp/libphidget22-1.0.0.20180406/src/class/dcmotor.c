/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/dcmotor.gen.h"
#include "class/dcmotor.gen.c"

double PhidgetDCMotor_getLastBrakingStrength(PhidgetDCMotorHandle);

static void
PhidgetDCMotor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDCMotor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDCMotorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDCMotorHandle)phid;

	switch (bp->vpkt) {
	case BP_SETDUTYCYCLE:
	case BP_SETBRAKINGDUTYCYCLE:
		TESTRANGE(getBridgePacketDouble(bp, 0), -ch->maxVelocity, ch->maxVelocity);
		res = _bridgeInput(phid, bp);
		break;

	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetDCMotor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

double
PhidgetDCMotor_getLastBrakingStrength(PhidgetDCMotorHandle ch) {
	return (ch->brakingStrength);
}