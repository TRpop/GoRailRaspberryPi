/* Generated: Fri Sep 16 2016 16:10:18 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/bldcmotor.gen.h"
#include "class/bldcmotor.gen.c"

double PhidgetBLDCMotor_getLastBrakingStrength(PhidgetBLDCMotorHandle);
int64_t PhidgetBLDCMotor_getLastPosition(PhidgetBLDCMotorHandle);

static void CCONV
PhidgetBLDCMotor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static PhidgetReturnCode CCONV
PhidgetBLDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetBLDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetBLDCMotor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetBLDCMotor_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetBLDCMotor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetBLDCMotorHandle)phid;

	switch (bp->vpkt) {
	case BP_SETBRAKINGDUTYCYCLE:
	case BP_SETDUTYCYCLE:
		TESTRANGE(getBridgePacketDouble(bp, 0), -ch->maxVelocity, ch->maxVelocity);
		res = _bridgeInput(phid, bp);
		break;
	case BP_POSITIONCHANGE:
		ch->position = getBridgePacketInt64(bp, 0);
		FIRECH(ch, PositionChange, (ch->position + ch->positionOffset) *ch->rescaleFactor);
		res = EPHIDGET_OK;
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void CCONV
PhidgetBLDCMotor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

API_PRETURN
PhidgetBLDCMotor_setRescaleFactor(PhidgetBLDCMotorHandle ch, double rescaleFactor) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	if (rescaleFactor == 0)
		return EPHIDGET_INVALIDARG;

	ch->rescaleFactor = rescaleFactor;
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getPosition(PhidgetBLDCMotorHandle ch, double *position) {

	TESTPTR(ch);
	TESTPTR(position);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*position = (ch->position + ch->positionOffset) * ch->rescaleFactor;
	if (ch->position == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinPosition(PhidgetBLDCMotorHandle ch, double *minPosition) {

	TESTPTR(ch);
	TESTPTR(minPosition);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*minPosition = (ch->minPosition + ch->positionOffset)* ch->rescaleFactor;
	if (ch->minPosition == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxPosition(PhidgetBLDCMotorHandle ch, double *maxPosition) {

	TESTPTR(ch);
	TESTPTR(maxPosition);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*maxPosition = (ch->maxPosition + ch->positionOffset) * ch->rescaleFactor;
	if (ch->maxPosition == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_addPositionOffset(PhidgetBLDCMotorHandle ch, double positionOffset) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	ch->positionOffset += roundl(positionOffset / ch->rescaleFactor);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setStallVelocity(PhidgetBLDCMotorHandle ch,
	double stallVelocity) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTALLVELOCITY, NULL, NULL, "%g",
		stallVelocity / ch->rescaleFactor));
}


API_PRETURN
PhidgetBLDCMotor_getStallVelocity(PhidgetBLDCMotorHandle ch, double *stallVelocity) {

	TESTPTR(ch);
	TESTPTR(stallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*stallVelocity = ch->stallVelocity * ch->rescaleFactor;
	if (ch->stallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinStallVelocity(PhidgetBLDCMotorHandle ch, double *minStallVelocity) {

	TESTPTR(ch);
	TESTPTR(minStallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*minStallVelocity = ch->minStallVelocity * ch->rescaleFactor;
	if (ch->minStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxStallVelocity(PhidgetBLDCMotorHandle ch, double *maxStallVelocity) {

	TESTPTR(ch);
	TESTPTR(maxStallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED(ch);

	*maxStallVelocity = ch->maxStallVelocity * ch->rescaleFactor;
	if (ch->maxStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

double
PhidgetBLDCMotor_getLastBrakingStrength(PhidgetBLDCMotorHandle ch) {
	return (ch->brakingStrength);
}

int64_t
PhidgetBLDCMotor_getLastPosition(PhidgetBLDCMotorHandle ch) {
	return (ch->position);
}
