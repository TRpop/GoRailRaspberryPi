/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetBLDCMotor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetBLDCMotor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetBLDCMotor {
	struct _PhidgetChannel phid;
	int64_t positionOffset;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	double brakingStrength;
	double minBrakingStrength;
	double maxBrakingStrength;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	int64_t position;
	int64_t minPosition;
	int64_t maxPosition;
	double rescaleFactor;
	double stallVelocity;
	double minStallVelocity;
	double maxStallVelocity;
	double targetBrakingStrength;
	double targetVelocity;
	double velocity;
	double minVelocity;
	double maxVelocity;
	PhidgetBLDCMotor_OnBrakingStrengthChangeCallback BrakingStrengthChange;
	void *BrakingStrengthChangeCtx;
	PhidgetBLDCMotor_OnPositionChangeCallback PositionChange;
	void *PositionChangeCtx;
	PhidgetBLDCMotor_OnVelocityUpdateCallback VelocityUpdate;
	void *VelocityUpdateCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetBLDCMotorHandle ch;
	int version;

	ch = (PhidgetBLDCMotorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->positionOffset = getBridgePacketInt64ByName(bp, "positionOffset");
	ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	ch->brakingStrength = getBridgePacketDoubleByName(bp, "brakingStrength");
	ch->minBrakingStrength = getBridgePacketDoubleByName(bp, "minBrakingStrength");
	ch->maxBrakingStrength = getBridgePacketDoubleByName(bp, "maxBrakingStrength");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->position = getBridgePacketInt64ByName(bp, "position");
	ch->minPosition = getBridgePacketInt64ByName(bp, "minPosition");
	ch->maxPosition = getBridgePacketInt64ByName(bp, "maxPosition");
	ch->rescaleFactor = getBridgePacketDoubleByName(bp, "rescaleFactor");
	ch->stallVelocity = getBridgePacketDoubleByName(bp, "stallVelocity");
	ch->minStallVelocity = getBridgePacketDoubleByName(bp, "minStallVelocity");
	ch->maxStallVelocity = getBridgePacketDoubleByName(bp, "maxStallVelocity");
	ch->targetBrakingStrength = getBridgePacketDoubleByName(bp, "targetBrakingStrength");
	ch->targetVelocity = getBridgePacketDoubleByName(bp, "targetVelocity");
	ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	ch->minVelocity = getBridgePacketDoubleByName(bp, "minVelocity");
	ch->maxVelocity = getBridgePacketDoubleByName(bp, "maxVelocity");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetBLDCMotorHandle ch;

	ch = (PhidgetBLDCMotorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",positionOffset=%l"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",brakingStrength=%g"
	  ",minBrakingStrength=%g"
	  ",maxBrakingStrength=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",position=%l"
	  ",minPosition=%l"
	  ",maxPosition=%l"
	  ",rescaleFactor=%g"
	  ",stallVelocity=%g"
	  ",minStallVelocity=%g"
	  ",maxStallVelocity=%g"
	  ",targetBrakingStrength=%g"
	  ",targetVelocity=%g"
	  ",velocity=%g"
	  ",minVelocity=%g"
	  ",maxVelocity=%g"
	  ,0 /* class version */
	  ,ch->positionOffset
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->brakingStrength
	  ,ch->minBrakingStrength
	  ,ch->maxBrakingStrength
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->rescaleFactor
	  ,ch->stallVelocity
	  ,ch->minStallVelocity
	  ,ch->maxStallVelocity
	  ,ch->targetBrakingStrength
	  ,ch->targetVelocity
	  ,ch->velocity
	  ,ch->minVelocity
	  ,ch->maxVelocity
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetBLDCMotorHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETACCELERATION:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minAcceleration, ch->maxAcceleration);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->acceleration = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Acceleration");
		break;
	case BP_SETDATAINTERVAL:
		TESTRANGE(getBridgePacketUInt32(bp, 0), ch->minDataInterval, ch->maxDataInterval);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->dataInterval = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "DataInterval");
		break;
	case BP_SETSTALLVELOCITY:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minStallVelocity, ch->maxStallVelocity);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->stallVelocity = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "StallVelocity");
		break;
	case BP_SETBRAKINGDUTYCYCLE:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->targetBrakingStrength = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "TargetBrakingStrength");
		break;
	case BP_SETDUTYCYCLE:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->targetVelocity = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "TargetVelocity");
		break;
	case BP_BRAKINGSTRENGTHCHANGE:
		ch->brakingStrength = getBridgePacketDouble(bp, 0);
		FIRECH(ch, BrakingStrengthChange, ch->brakingStrength);
		break;
	case BP_DUTYCYCLECHANGE:
		ch->velocity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, VelocityUpdate, ch->velocity);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetBLDCMotorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxAcceleration = 100;
		ch->maxBrakingStrength = 1;
		ch->maxVelocity = 1;
		ch->maxPosition = 1000000000000000;
		ch->minVelocity = 0;
		ch->minAcceleration = 0.1;
		ch->minBrakingStrength = 0;
		ch->minPosition = -1000000000000000;
		ch->position = 0;
		ch->rescaleFactor = 1;
		ch->targetVelocity = PUNK_DBL;
		ch->velocity = PUNK_DBL;
		ch->brakingStrength = PUNK_DBL;
		ch->stallVelocity = 1000;
		ch->minStallVelocity = 0;
		ch->maxStallVelocity = 2000;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->positionOffset = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetBLDCMotorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, "%g",
		  ch->targetBrakingStrength);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [targetBrakingStrength] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSTALLVELOCITY, NULL, NULL, "%g", ch->stallVelocity);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [stallVelocity] default: %d", phid, ret);
			break;
		}
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;

	ch = (PhidgetBLDCMotorHandle)phid;

	if(ch->brakingStrength != PUNK_DBL)
		FIRECH(ch, BrakingStrengthChange, ch->brakingStrength);
	if(ch->velocity != PUNK_DBL)
		FIRECH(ch, VelocityUpdate, ch->velocity);

}

static void CCONV
PhidgetBLDCMotor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetBLDCMotor));
}

API_PRETURN
PhidgetBLDCMotor_create(PhidgetBLDCMotorHandle *phidp) {

	CHANNELCREATE_BODY(BLDCMotor, PHIDCHCLASS_BLDCMOTOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_delete(PhidgetBLDCMotorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetBLDCMotor_setAcceleration(PhidgetBLDCMotorHandle ch, double acceleration) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, "%g",
	  acceleration));
}

API_PRETURN
PhidgetBLDCMotor_getAcceleration(PhidgetBLDCMotorHandle ch, double *acceleration) {

	TESTPTRL(ch);
	TESTPTRL(acceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*acceleration = ch->acceleration;
	if (ch->acceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinAcceleration(PhidgetBLDCMotorHandle ch, double *minAcceleration) {

	TESTPTRL(ch);
	TESTPTRL(minAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*minAcceleration = ch->minAcceleration;
	if (ch->minAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxAcceleration(PhidgetBLDCMotorHandle ch, double *maxAcceleration) {

	TESTPTRL(ch);
	TESTPTRL(maxAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*maxAcceleration = ch->maxAcceleration;
	if (ch->maxAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getBrakingStrength(PhidgetBLDCMotorHandle ch, double *brakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(brakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*brakingStrength = ch->brakingStrength;
	if (ch->brakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinBrakingStrength(PhidgetBLDCMotorHandle ch, double *minBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(minBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*minBrakingStrength = ch->minBrakingStrength;
	if (ch->minBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxBrakingStrength(PhidgetBLDCMotorHandle ch, double *maxBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(maxBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*maxBrakingStrength = ch->maxBrakingStrength;
	if (ch->maxBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setDataInterval(PhidgetBLDCMotorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetBLDCMotor_getDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getRescaleFactor(PhidgetBLDCMotorHandle ch, double *rescaleFactor) {

	TESTPTRL(ch);
	TESTPTRL(rescaleFactor);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*rescaleFactor = ch->rescaleFactor;
	if (ch->rescaleFactor == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setTargetBrakingStrength(PhidgetBLDCMotorHandle ch, double targetBrakingStrength) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, "%g",
	  targetBrakingStrength));
}

API_PRETURN
PhidgetBLDCMotor_getTargetBrakingStrength(PhidgetBLDCMotorHandle ch, double *targetBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(targetBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*targetBrakingStrength = ch->targetBrakingStrength;
	if (ch->targetBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setTargetVelocity(PhidgetBLDCMotorHandle ch, double targetVelocity) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g",
	  targetVelocity));
}

API_PRETURN
PhidgetBLDCMotor_getTargetVelocity(PhidgetBLDCMotorHandle ch, double *targetVelocity) {

	TESTPTRL(ch);
	TESTPTRL(targetVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*targetVelocity = ch->targetVelocity;
	if (ch->targetVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getVelocity(PhidgetBLDCMotorHandle ch, double *velocity) {

	TESTPTRL(ch);
	TESTPTRL(velocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*velocity = ch->velocity;
	if (ch->velocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinVelocity(PhidgetBLDCMotorHandle ch, double *minVelocity) {

	TESTPTRL(ch);
	TESTPTRL(minVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*minVelocity = ch->minVelocity;
	if (ch->minVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxVelocity(PhidgetBLDCMotorHandle ch, double *maxVelocity) {

	TESTPTRL(ch);
	TESTPTRL(maxVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHEDL(ch);

	*maxVelocity = ch->maxVelocity;
	if (ch->maxVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnBrakingStrengthChangeHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnBrakingStrengthChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);

	ch->BrakingStrengthChange = fptr;
	ch->BrakingStrengthChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnPositionChangeHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);

	ch->PositionChange = fptr;
	ch->PositionChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnVelocityUpdateHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnVelocityUpdateCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_BLDCMOTOR);

	ch->VelocityUpdate = fptr;
	ch->VelocityUpdateCtx = ctx;

	return (EPHIDGET_OK);
}
