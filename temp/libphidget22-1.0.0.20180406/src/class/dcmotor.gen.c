/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/motorcontroldevice.h"
static void CCONV PhidgetDCMotor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetDCMotor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDCMotor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDCMotor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDCMotor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetDCMotor {
	struct _PhidgetChannel phid;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	double backEMF;
	int backEMFSensingState;
	double brakingStrength;
	double minBrakingStrength;
	double maxBrakingStrength;
	double currentLimit;
	double minCurrentLimit;
	double maxCurrentLimit;
	double currentRegulatorGain;
	double minCurrentRegulatorGain;
	double maxCurrentRegulatorGain;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	Phidget_FanMode fanMode;
	double targetBrakingStrength;
	double targetVelocity;
	double velocity;
	double minVelocity;
	double maxVelocity;
	PhidgetDCMotor_OnBackEMFChangeCallback BackEMFChange;
	void *BackEMFChangeCtx;
	PhidgetDCMotor_OnBrakingStrengthChangeCallback BrakingStrengthChange;
	void *BrakingStrengthChangeCtx;
	PhidgetDCMotor_OnVelocityUpdateCallback VelocityUpdate;
	void *VelocityUpdateCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDCMotorHandle ch;
	int version;

	ch = (PhidgetDCMotorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	ch->backEMF = getBridgePacketDoubleByName(bp, "backEMF");
	ch->backEMFSensingState = getBridgePacketInt32ByName(bp, "backEMFSensingState");
	ch->brakingStrength = getBridgePacketDoubleByName(bp, "brakingStrength");
	ch->minBrakingStrength = getBridgePacketDoubleByName(bp, "minBrakingStrength");
	ch->maxBrakingStrength = getBridgePacketDoubleByName(bp, "maxBrakingStrength");
	ch->currentLimit = getBridgePacketDoubleByName(bp, "currentLimit");
	ch->minCurrentLimit = getBridgePacketDoubleByName(bp, "minCurrentLimit");
	ch->maxCurrentLimit = getBridgePacketDoubleByName(bp, "maxCurrentLimit");
	ch->currentRegulatorGain = getBridgePacketDoubleByName(bp, "currentRegulatorGain");
	ch->minCurrentRegulatorGain = getBridgePacketDoubleByName(bp, "minCurrentRegulatorGain");
	ch->maxCurrentRegulatorGain = getBridgePacketDoubleByName(bp, "maxCurrentRegulatorGain");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->fanMode = getBridgePacketInt32ByName(bp, "fanMode");
	ch->targetBrakingStrength = getBridgePacketDoubleByName(bp, "targetBrakingStrength");
	ch->targetVelocity = getBridgePacketDoubleByName(bp, "targetVelocity");
	ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	ch->minVelocity = getBridgePacketDoubleByName(bp, "minVelocity");
	ch->maxVelocity = getBridgePacketDoubleByName(bp, "maxVelocity");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDCMotorHandle ch;

	ch = (PhidgetDCMotorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",backEMF=%g"
	  ",backEMFSensingState=%d"
	  ",brakingStrength=%g"
	  ",minBrakingStrength=%g"
	  ",maxBrakingStrength=%g"
	  ",currentLimit=%g"
	  ",minCurrentLimit=%g"
	  ",maxCurrentLimit=%g"
	  ",currentRegulatorGain=%g"
	  ",minCurrentRegulatorGain=%g"
	  ",maxCurrentRegulatorGain=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",fanMode=%d"
	  ",targetBrakingStrength=%g"
	  ",targetVelocity=%g"
	  ",velocity=%g"
	  ",minVelocity=%g"
	  ",maxVelocity=%g"
	  ,0 /* class version */
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->backEMF
	  ,ch->backEMFSensingState
	  ,ch->brakingStrength
	  ,ch->minBrakingStrength
	  ,ch->maxBrakingStrength
	  ,ch->currentLimit
	  ,ch->minCurrentLimit
	  ,ch->maxCurrentLimit
	  ,ch->currentRegulatorGain
	  ,ch->minCurrentRegulatorGain
	  ,ch->maxCurrentRegulatorGain
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->fanMode
	  ,ch->targetBrakingStrength
	  ,ch->targetVelocity
	  ,ch->velocity
	  ,ch->minVelocity
	  ,ch->maxVelocity
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDCMotorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDCMotorHandle)phid;
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
	case BP_SETBACKEMFSENSINGSTATE:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->backEMFSensingState = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "BackEMFSensingState");
		break;
	case BP_SETCURRENTLIMIT:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minCurrentLimit, ch->maxCurrentLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->currentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "CurrentLimit");
		break;
	case BP_SETCURRENTREGULATORGAIN:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minCurrentRegulatorGain,
		  ch->maxCurrentRegulatorGain);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->currentRegulatorGain = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "CurrentRegulatorGain");
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
	case BP_SETFANMODE:
		if (!supportedFanMode(phid, (Phidget_FanMode)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->fanMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "FanMode");
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
	case BP_BACKEMFCHANGE:
		ch->backEMF = getBridgePacketDouble(bp, 0);
		FIRECH(ch, BackEMFChange, ch->backEMF);
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
	PhidgetMotorControlDeviceHandle parentMotorControl;
	PhidgetDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetDCMotorHandle)phid;

	ret = EPHIDGET_OK;

	parentMotorControl = (PhidgetMotorControlDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 64;
		ch->maxDataInterval = 60000;
		ch->targetVelocity = 0;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 12.45;
		ch->minAcceleration = 0.0122;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->velocity = 0;
		break;
	case PHIDCHUID_1064_DCMOTOR_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->targetVelocity = 0;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 19.4;
		ch->minAcceleration = 0.019;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->velocity = 0;
		break;
	case PHIDCHUID_1065_DCMOTOR_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->targetVelocity = 0;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 62.5;
		ch->minAcceleration = 0.2778;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		ch->backEMFSensingState = 0;
		ch->backEMF = parentMotorControl->backEMF[ch->phid.index];
		ch->velocity = 0;
		break;
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 25;
		ch->minCurrentLimit = 2;
		ch->currentRegulatorGain = 10;
		ch->maxCurrentRegulatorGain = 100;
		ch->minCurrentRegulatorGain = 1;
		ch->targetVelocity = PUNK_DBL;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 100;
		ch->minAcceleration = 0.1;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		ch->fanMode = FAN_MODE_AUTO;
		ch->velocity = 0;
		ch->brakingStrength = PUNK_DBL;
		break;
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 25;
		ch->minCurrentLimit = 2;
		ch->currentRegulatorGain = 10;
		ch->maxCurrentRegulatorGain = 100;
		ch->minCurrentRegulatorGain = 1;
		ch->targetVelocity = PUNK_DBL;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 100;
		ch->minAcceleration = 0.1;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		ch->fanMode = FAN_MODE_AUTO;
		ch->velocity = 0;
		ch->brakingStrength = PUNK_DBL;
		break;
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 2;
		ch->minCurrentLimit = 0;
		ch->targetVelocity = PUNK_DBL;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 100;
		ch->minAcceleration = 0.1;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		ch->velocity = 0;
		ch->brakingStrength = PUNK_DBL;
		break;
	case PHIDCHUID_MCSML1_DCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 4;
		ch->minCurrentLimit = 0;
		ch->targetVelocity = PUNK_DBL;
		ch->maxVelocity = 1;
		ch->minVelocity = 0;
		ch->maxAcceleration = 100;
		ch->minAcceleration = 0.1;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		ch->velocity = 0;
		ch->brakingStrength = PUNK_DBL;
		break;
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		ch->maxCurrentLimit = 3.5;
		ch->minCurrentLimit = 0;
		ch->targetVelocity = PUNK_DBL;
		ch->maxVelocity = 1;
		ch->minVelocity = -1;
		ch->maxAcceleration = 100;
		ch->minAcceleration = 0.1;
		ch->acceleration = PUNK_DBL;
		ch->targetBrakingStrength = PUNK_DBL;
		ch->maxBrakingStrength = 1;
		ch->minBrakingStrength = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetDCMotorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
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
		break;
	case PHIDCHUID_1064_DCMOTOR_100:
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
		break;
	case PHIDCHUID_1065_DCMOTOR_100:
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
		ret = bridgeSendToDevice(phid, BP_SETBACKEMFSENSINGSTATE, NULL, NULL, "%d",
		  ch->backEMFSensingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [backEMFSensingState] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DCC1000_DCMOTOR_100:
		break;
	case PHIDCHUID_DCC1000_DCMOTOR_200:
		break;
	case PHIDCHUID_DCC1001_DCMOTOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [currentLimit] default: %d", phid, ret);
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
		break;
	case PHIDCHUID_MCSML1_DCMOTOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [currentLimit] default: %d", phid, ret);
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
		break;
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetDCMotorHandle ch;

	ch = (PhidgetDCMotorHandle)phid;

	if(ch->backEMF != PUNK_DBL)
		FIRECH(ch, BackEMFChange, ch->backEMF);
	if(ch->brakingStrength != PUNK_DBL)
		FIRECH(ch, BrakingStrengthChange, ch->brakingStrength);
	if(ch->velocity != PUNK_DBL)
		FIRECH(ch, VelocityUpdate, ch->velocity);

}

static void CCONV
PhidgetDCMotor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDCMotor));
}

API_PRETURN
PhidgetDCMotor_create(PhidgetDCMotorHandle *phidp) {

	CHANNELCREATE_BODY(DCMotor, PHIDCHCLASS_DCMOTOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_delete(PhidgetDCMotorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDCMotor_setAcceleration(PhidgetDCMotorHandle ch, double acceleration) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, "%g",
	  acceleration));
}

API_PRETURN
PhidgetDCMotor_getAcceleration(PhidgetDCMotorHandle ch, double *acceleration) {

	TESTPTRL(ch);
	TESTPTRL(acceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*acceleration = ch->acceleration;
	if (ch->acceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinAcceleration(PhidgetDCMotorHandle ch, double *minAcceleration) {

	TESTPTRL(ch);
	TESTPTRL(minAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*minAcceleration = ch->minAcceleration;
	if (ch->minAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxAcceleration(PhidgetDCMotorHandle ch, double *maxAcceleration) {

	TESTPTRL(ch);
	TESTPTRL(maxAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*maxAcceleration = ch->maxAcceleration;
	if (ch->maxAcceleration == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getBackEMF(PhidgetDCMotorHandle ch, double *backEMF) {

	TESTPTRL(ch);
	TESTPTRL(backEMF);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_DCC1000_DCMOTOR_100:
	case PHIDCHUID_DCC1000_DCMOTOR_200:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*backEMF = ch->backEMF;
	if (ch->backEMF == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setBackEMFSensingState(PhidgetDCMotorHandle ch, int backEMFSensingState) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBACKEMFSENSINGSTATE, NULL, NULL, "%d",
	  backEMFSensingState));
}

API_PRETURN
PhidgetDCMotor_getBackEMFSensingState(PhidgetDCMotorHandle ch, int *backEMFSensingState) {

	TESTPTRL(ch);
	TESTPTRL(backEMFSensingState);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_DCC1000_DCMOTOR_100:
	case PHIDCHUID_DCC1000_DCMOTOR_200:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*backEMFSensingState = ch->backEMFSensingState;
	if (ch->backEMFSensingState == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getBrakingStrength(PhidgetDCMotorHandle ch, double *brakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(brakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*brakingStrength = ch->brakingStrength;
	if (ch->brakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinBrakingStrength(PhidgetDCMotorHandle ch, double *minBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(minBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minBrakingStrength = ch->minBrakingStrength;
	if (ch->minBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxBrakingStrength(PhidgetDCMotorHandle ch, double *maxBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(maxBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxBrakingStrength = ch->maxBrakingStrength;
	if (ch->maxBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setCurrentLimit(PhidgetDCMotorHandle ch, double currentLimit) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTLIMIT, NULL, NULL, "%g",
	  currentLimit));
}

API_PRETURN
PhidgetDCMotor_getCurrentLimit(PhidgetDCMotorHandle ch, double *currentLimit) {

	TESTPTRL(ch);
	TESTPTRL(currentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*currentLimit = ch->currentLimit;
	if (ch->currentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinCurrentLimit(PhidgetDCMotorHandle ch, double *minCurrentLimit) {

	TESTPTRL(ch);
	TESTPTRL(minCurrentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minCurrentLimit = ch->minCurrentLimit;
	if (ch->minCurrentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxCurrentLimit(PhidgetDCMotorHandle ch, double *maxCurrentLimit) {

	TESTPTRL(ch);
	TESTPTRL(maxCurrentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxCurrentLimit = ch->maxCurrentLimit;
	if (ch->maxCurrentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setCurrentRegulatorGain(PhidgetDCMotorHandle ch, double currentRegulatorGain) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTREGULATORGAIN, NULL, NULL, "%g",
	  currentRegulatorGain));
}

API_PRETURN
PhidgetDCMotor_getCurrentRegulatorGain(PhidgetDCMotorHandle ch, double *currentRegulatorGain) {

	TESTPTRL(ch);
	TESTPTRL(currentRegulatorGain);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*currentRegulatorGain = ch->currentRegulatorGain;
	if (ch->currentRegulatorGain == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinCurrentRegulatorGain(PhidgetDCMotorHandle ch, double *minCurrentRegulatorGain) {

	TESTPTRL(ch);
	TESTPTRL(minCurrentRegulatorGain);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minCurrentRegulatorGain = ch->minCurrentRegulatorGain;
	if (ch->minCurrentRegulatorGain == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxCurrentRegulatorGain(PhidgetDCMotorHandle ch, double *maxCurrentRegulatorGain) {

	TESTPTRL(ch);
	TESTPTRL(maxCurrentRegulatorGain);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxCurrentRegulatorGain = ch->maxCurrentRegulatorGain;
	if (ch->maxCurrentRegulatorGain == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setDataInterval(PhidgetDCMotorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetDCMotor_getDataInterval(PhidgetDCMotorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinDataInterval(PhidgetDCMotorHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxDataInterval(PhidgetDCMotorHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setFanMode(PhidgetDCMotorHandle ch, Phidget_FanMode fanMode) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFANMODE, NULL, NULL, "%d", fanMode));
}

API_PRETURN
PhidgetDCMotor_getFanMode(PhidgetDCMotorHandle ch, Phidget_FanMode *fanMode) {

	TESTPTRL(ch);
	TESTPTRL(fanMode);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1060_DCMOTOR_100:
	case PHIDCHUID_1064_DCMOTOR_100:
	case PHIDCHUID_1065_DCMOTOR_100:
	case PHIDCHUID_DCC1001_DCMOTOR_100:
	case PHIDCHUID_MCSML1_DCMOTOR_100:
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*fanMode = ch->fanMode;
	if (ch->fanMode == (Phidget_FanMode)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setTargetBrakingStrength(PhidgetDCMotorHandle ch, double targetBrakingStrength) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, "%g",
	  targetBrakingStrength));
}

API_PRETURN
PhidgetDCMotor_getTargetBrakingStrength(PhidgetDCMotorHandle ch, double *targetBrakingStrength) {

	TESTPTRL(ch);
	TESTPTRL(targetBrakingStrength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*targetBrakingStrength = ch->targetBrakingStrength;
	if (ch->targetBrakingStrength == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setTargetVelocity(PhidgetDCMotorHandle ch, double targetVelocity) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g",
	  targetVelocity));
}

API_PRETURN
PhidgetDCMotor_getTargetVelocity(PhidgetDCMotorHandle ch, double *targetVelocity) {

	TESTPTRL(ch);
	TESTPTRL(targetVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*targetVelocity = ch->targetVelocity;
	if (ch->targetVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getVelocity(PhidgetDCMotorHandle ch, double *velocity) {

	TESTPTRL(ch);
	TESTPTRL(velocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S4309R_DCMOTOR_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*velocity = ch->velocity;
	if (ch->velocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMinVelocity(PhidgetDCMotorHandle ch, double *minVelocity) {

	TESTPTRL(ch);
	TESTPTRL(minVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*minVelocity = ch->minVelocity;
	if (ch->minVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_getMaxVelocity(PhidgetDCMotorHandle ch, double *maxVelocity) {

	TESTPTRL(ch);
	TESTPTRL(maxVelocity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);
	TESTATTACHEDL(ch);

	*maxVelocity = ch->maxVelocity;
	if (ch->maxVelocity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setOnBackEMFChangeHandler(PhidgetDCMotorHandle ch,
  PhidgetDCMotor_OnBackEMFChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);

	ch->BackEMFChange = fptr;
	ch->BackEMFChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setOnBrakingStrengthChangeHandler(PhidgetDCMotorHandle ch,
  PhidgetDCMotor_OnBrakingStrengthChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);

	ch->BrakingStrengthChange = fptr;
	ch->BrakingStrengthChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDCMotor_setOnVelocityUpdateHandler(PhidgetDCMotorHandle ch,
  PhidgetDCMotor_OnVelocityUpdateCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DCMOTOR);

	ch->VelocityUpdate = fptr;
	ch->VelocityUpdateCtx = ctx;

	return (EPHIDGET_OK);
}
