/* Generated: Thu Nov 24 2016 13:44:03 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/motorpositioncontroller.gen.h"
#include "class/motorpositioncontroller.gen.c"

int64_t PhidgetMotorPositionController_getLastPosition(PhidgetMotorPositionControllerHandle);

static void CCONV
PhidgetMotorPositionController_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
	PhidgetMotorPositionControllerHandle ch;
	
	ch = (PhidgetMotorPositionControllerHandle)phid;

	switch (code) {
	case EEPHIDGET_MOTORSTALL:
		ch->engaged = 0;
		break;
	default:
		break;
	}
}

static PhidgetReturnCode CCONV
PhidgetMotorPositionController_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMotorPositionController_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetMotorPositionController_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetMotorPositionController_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetMotorPositionController_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetMotorPositionControllerHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetMotorPositionControllerHandle)phid;

	switch (bp->vpkt) {
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
PhidgetMotorPositionController_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

API_PRETURN
PhidgetMotorPositionController_addPositionOffset(PhidgetMotorPositionControllerHandle ch,
  double positionOffset) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	ch->positionOffset += roundl(positionOffset / ch->rescaleFactor);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setAcceleration(PhidgetMotorPositionControllerHandle ch,
  double acceleration) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, "%g",
		acceleration / ch->rescaleFactor));
}

API_PRETURN
PhidgetMotorPositionController_getAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *acceleration) {

	TESTPTR(ch);
	TESTPTR(acceleration);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*acceleration = ch->acceleration * ch->rescaleFactor;
	if (ch->acceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *minAcceleration) {

	TESTPTR(ch);
	TESTPTR(minAcceleration);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*minAcceleration = ch->minAcceleration * ch->rescaleFactor;
	if (ch->minAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *maxAcceleration) {

	TESTPTR(ch);
	TESTPTR(maxAcceleration);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*maxAcceleration = ch->maxAcceleration * ch->rescaleFactor;
	if (ch->maxAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setDeadBand(PhidgetMotorPositionControllerHandle ch, double deadBand) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDEADBAND, NULL, NULL, "%u",
		roundu((deadBand / ch->rescaleFactor))));
}

API_PRETURN
PhidgetMotorPositionController_getDeadBand(PhidgetMotorPositionControllerHandle ch, double *deadBand) {

	TESTPTR(ch);
	TESTPTR(deadBand);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*deadBand = ch->deadBand * ch->rescaleFactor;
	if (ch->deadBand == PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxVelocityLimit(PhidgetMotorPositionControllerHandle ch,
  double *maxVelocityLimit) {

	TESTPTR(ch);
	TESTPTR(maxVelocityLimit);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*maxVelocityLimit = ch->maxVelocityLimit * ch->rescaleFactor;
	if (ch->maxVelocityLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getPosition(PhidgetMotorPositionControllerHandle ch, double *position) {

	TESTPTR(ch);
	TESTPTR(position);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*position = (ch->position + ch->positionOffset) * ch->rescaleFactor;
	if (ch->position == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinPosition(PhidgetMotorPositionControllerHandle ch,
  double *minPosition) {

	TESTPTR(ch);
	TESTPTR(minPosition);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*minPosition = (ch->minPosition + ch->positionOffset)* ch->rescaleFactor;
	if (ch->minPosition == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxPosition(PhidgetMotorPositionControllerHandle ch,
  double *maxPosition) {

	TESTPTR(ch);
	TESTPTR(maxPosition);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*maxPosition = (ch->maxPosition + ch->positionOffset) * ch->rescaleFactor;
	if (ch->maxPosition == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setRescaleFactor(PhidgetMotorPositionControllerHandle ch,
  double rescaleFactor) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	if (rescaleFactor == 0)
		return EPHIDGET_INVALIDARG;

	ch->rescaleFactor = rescaleFactor;
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinVelocityLimit(PhidgetMotorPositionControllerHandle ch,
  double *minVelocityLimit) {

	TESTPTR(ch);
	TESTPTR(minVelocityLimit);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*minVelocityLimit = ch->minVelocityLimit;
	if (ch->minVelocityLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setTargetPosition(PhidgetMotorPositionControllerHandle ch,
  double targetPosition) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTARGETPOSITION, NULL, NULL, "%l",
		roundl((targetPosition / ch->rescaleFactor) - ch->positionOffset)));
}

API_VRETURN
PhidgetMotorPositionController_setTargetPosition_async(PhidgetMotorPositionControllerHandle ch,
  double targetPosition, Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_MOTORPOSITIONCONTROLLER) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTARGETPOSITION, fptr, ctx, "%l", roundl((targetPosition / ch->rescaleFactor) - ch->positionOffset));
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetMotorPositionController_getTargetPosition(PhidgetMotorPositionControllerHandle ch,
  double *targetPosition) {

	TESTPTR(ch);
	TESTPTR(targetPosition);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*targetPosition = (ch->targetPosition + ch->positionOffset) * ch->rescaleFactor;
	if (ch->targetPosition == PUNK_INT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setVelocityLimit(PhidgetMotorPositionControllerHandle ch, double velocityLimit) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g",
		velocityLimit / ch->rescaleFactor));
}

API_PRETURN
PhidgetMotorPositionController_getVelocityLimit(PhidgetMotorPositionControllerHandle ch, double *velocityLimit) {

	TESTPTR(ch);
	TESTPTR(velocityLimit);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*velocityLimit = ch->velocityLimit * ch->rescaleFactor;
	if (ch->velocityLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setStallVelocity(PhidgetMotorPositionControllerHandle ch,
	double stallVelocity) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTALLVELOCITY, NULL, NULL, "%g",
		stallVelocity / ch->rescaleFactor));
}

API_PRETURN
PhidgetMotorPositionController_getStallVelocity(PhidgetMotorPositionControllerHandle ch, double *stallVelocity) {

	TESTPTR(ch);
	TESTPTR(stallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*stallVelocity = ch->stallVelocity * ch->rescaleFactor;
	if (ch->stallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinStallVelocity(PhidgetMotorPositionControllerHandle ch, double *minStallVelocity) {

	TESTPTR(ch);
	TESTPTR(minStallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*minStallVelocity = ch->minStallVelocity * ch->rescaleFactor;
	if (ch->minStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxStallVelocity(PhidgetMotorPositionControllerHandle ch, double *maxStallVelocity) {

	TESTPTR(ch);
	TESTPTR(maxStallVelocity);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED(ch);

	*maxStallVelocity = ch->maxStallVelocity * ch->rescaleFactor;
	if (ch->maxStallVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

int64_t
PhidgetMotorPositionController_getLastPosition(PhidgetMotorPositionControllerHandle ch) {
	return (ch->position);
}
