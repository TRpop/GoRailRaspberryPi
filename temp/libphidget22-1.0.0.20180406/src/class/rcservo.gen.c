/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/servodevice.h"
#include "device/advancedservodevice.h"
static void CCONV PhidgetRCServo_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetRCServo_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetRCServo_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetRCServo_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetRCServo_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetRCServo_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetRCServo_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetRCServo {
	struct _PhidgetChannel phid;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	int engaged;
	int isMoving;
	double position;
	double minPosition;
	double maxPosition;
	double minPulseWidth;
	double maxPulseWidth;
	double minPulseWidthLimit;
	double maxPulseWidthLimit;
	int speedRampingState;
	double targetPosition;
	double torque;
	double minTorque;
	double maxTorque;
	double velocity;
	double velocityLimit;
	double minVelocityLimit;
	double maxVelocityLimit;
	PhidgetRCServo_Voltage voltage;
	PhidgetRCServo_OnPositionChangeCallback PositionChange;
	void *PositionChangeCtx;
	PhidgetRCServo_OnTargetPositionReachedCallback TargetPositionReached;
	void *TargetPositionReachedCtx;
	PhidgetRCServo_OnVelocityChangeCallback VelocityChange;
	void *VelocityChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRCServoHandle ch;
	int version;

	ch = (PhidgetRCServoHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 3) {
		logerr("%"PRIphid": bad version %d != 3", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->engaged = getBridgePacketInt32ByName(bp, "engaged");
	ch->isMoving = getBridgePacketInt32ByName(bp, "isMoving");
	ch->position = getBridgePacketDoubleByName(bp, "position");
	ch->minPosition = getBridgePacketDoubleByName(bp, "minPosition");
	ch->maxPosition = getBridgePacketDoubleByName(bp, "maxPosition");
	ch->minPulseWidth = getBridgePacketDoubleByName(bp, "minPulseWidth");
	ch->maxPulseWidth = getBridgePacketDoubleByName(bp, "maxPulseWidth");
	ch->minPulseWidthLimit = getBridgePacketDoubleByName(bp, "minPulseWidthLimit");
	ch->maxPulseWidthLimit = getBridgePacketDoubleByName(bp, "maxPulseWidthLimit");
	ch->speedRampingState = getBridgePacketInt32ByName(bp, "speedRampingState");
	ch->targetPosition = getBridgePacketDoubleByName(bp, "targetPosition");
	ch->torque = getBridgePacketDoubleByName(bp, "torque");
	ch->minTorque = getBridgePacketDoubleByName(bp, "minTorque");
	ch->maxTorque = getBridgePacketDoubleByName(bp, "maxTorque");
	ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	ch->velocityLimit = getBridgePacketDoubleByName(bp, "velocityLimit");
	ch->minVelocityLimit = getBridgePacketDoubleByName(bp, "minVelocityLimit");
	ch->maxVelocityLimit = getBridgePacketDoubleByName(bp, "maxVelocityLimit");
	ch->voltage = getBridgePacketInt32ByName(bp, "voltage");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetRCServoHandle ch;

	ch = (PhidgetRCServoHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",engaged=%d"
	  ",isMoving=%d"
	  ",position=%g"
	  ",minPosition=%g"
	  ",maxPosition=%g"
	  ",minPulseWidth=%g"
	  ",maxPulseWidth=%g"
	  ",minPulseWidthLimit=%g"
	  ",maxPulseWidthLimit=%g"
	  ",speedRampingState=%d"
	  ",targetPosition=%g"
	  ",torque=%g"
	  ",minTorque=%g"
	  ",maxTorque=%g"
	  ",velocity=%g"
	  ",velocityLimit=%g"
	  ",minVelocityLimit=%g"
	  ",maxVelocityLimit=%g"
	  ",voltage=%d"
	  ,3 /* class version */
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->engaged
	  ,ch->isMoving
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->minPulseWidth
	  ,ch->maxPulseWidth
	  ,ch->minPulseWidthLimit
	  ,ch->maxPulseWidthLimit
	  ,ch->speedRampingState
	  ,ch->targetPosition
	  ,ch->torque
	  ,ch->minTorque
	  ,ch->maxTorque
	  ,ch->velocity
	  ,ch->velocityLimit
	  ,ch->minVelocityLimit
	  ,ch->maxVelocityLimit
	  ,ch->voltage
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetRCServoHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetRCServoHandle)phid;
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
	case BP_SETENGAGED:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->engaged = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Engaged");
		break;
	case BP_SETMINPULSEWIDTH:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->minPulseWidth = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "MinPulseWidth");
		break;
	case BP_SETMAXPULSEWIDTH:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->maxPulseWidth = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "MaxPulseWidth");
		break;
	case BP_SETSPEEDRAMPINGSTATE:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->speedRampingState = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "SpeedRampingState");
		break;
	case BP_SETTARGETPOSITION:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->targetPosition = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "TargetPosition");
		break;
	case BP_SETDUTYCYCLE:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minTorque, ch->maxTorque);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->torque = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Torque");
		break;
	case BP_SETVELOCITYLIMIT:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minVelocityLimit, ch->maxVelocityLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->velocityLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "VelocityLimit");
		break;
	case BP_SETVOLTAGE:
		if (!supportedRCServoVoltage(phid, (PhidgetRCServo_Voltage)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->voltage = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Voltage");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetAdvancedServoDeviceHandle parentAdvancedServo;
	PhidgetServoDeviceHandle parentServo;
	PhidgetRCServoHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetRCServoHandle)phid;

	ret = EPHIDGET_OK;

	parentAdvancedServo = (PhidgetAdvancedServoDeviceHandle)phid->parent;
	parentServo = (PhidgetServoDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_300:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1000_RCSERVO_313:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		ch->maxPulseWidthLimit = 2550;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_313:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1001_RCSERVO_400:
		ch->maxPulseWidthLimit = 4095;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->minPulseWidthLimit = 1;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->position = parentServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->engaged = 0;
		ch->speedRampingState = 0;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_1061_RCSERVO_100:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		break;
	case PHIDCHUID_1061_RCSERVO_200:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		break;
	case PHIDCHUID_1061_RCSERVO_300:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		break;
	case PHIDCHUID_1066_RCSERVO_100:
		ch->maxPulseWidthLimit = 2730.666;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 68266.666;
		ch->maxAcceleration = 3413333.333;
		ch->minPulseWidthLimit = 0.083;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 208.334;
		ch->position = parentAdvancedServo->position[ch->phid.index];
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->velocity = parentAdvancedServo->velocity[ch->phid.index];
		ch->acceleration = 3800;
		ch->isMoving = parentAdvancedServo->isMoving[ch->phid.index];
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 32;
		break;
	case PHIDCHUID_RCC1000_RCSERVO_100:
		ch->maxPulseWidthLimit = 4000;
		ch->maxPulseWidth = 2450;
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 781250;
		ch->maxAcceleration = 3906250;
		ch->minPulseWidthLimit = 0.063;
		ch->minPulseWidth = 550;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 156.25;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 1900;
		ch->acceleration = 3800;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->voltage = RCSERVO_VOLTAGE_5V;
		break;
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 261888000;
		ch->maxAcceleration = 261888000;
		ch->maxTorque = 1;
		ch->minTorque = 0;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 10000;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 222605;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 445210;
		ch->torque = PUNK_DBL;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 20;
		break;
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 261888000;
		ch->maxAcceleration = 261888000;
		ch->maxTorque = 1;
		ch->minTorque = 0;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 10000;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 222605;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 445210;
		ch->torque = PUNK_DBL;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 20;
		break;
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		ch->maxPosition = 180;
		ch->maxVelocityLimit = 261888000;
		ch->maxAcceleration = 261888000;
		ch->maxTorque = 1;
		ch->minTorque = 0;
		ch->minPosition = 0;
		ch->minVelocityLimit = 0;
		ch->minAcceleration = 10000;
		ch->position = PUNK_DBL;
		ch->targetPosition = PUNK_DBL;
		ch->velocityLimit = 222605;
		ch->velocity = PUNK_DBL;
		ch->acceleration = 445210;
		ch->torque = PUNK_DBL;
		ch->isMoving = PUNK_BOOL;
		ch->engaged = 0;
		ch->speedRampingState = 1;
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->minDataInterval = 20;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetRCServoHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetRCServoHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1000_RCSERVO_300:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1000_RCSERVO_313:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1001_RCSERVO_313:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1001_RCSERVO_400:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1061_RCSERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1061_RCSERVO_200:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1061_RCSERVO_300:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1066_RCSERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g", ch->maxPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [maxPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETMINPULSEWIDTH, NULL, NULL, "%g", ch->minPulseWidth);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [minPulseWidth] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_RCC1000_RCSERVO_100:
		break;
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, "%d", ch->engaged);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [engaged] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, "%d", ch->engaged);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [engaged] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		ret = bridgeSendToDevice(phid, BP_SETVELOCITYLIMIT, NULL, NULL, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [velocityLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [acceleration] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, "%d", ch->engaged);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [engaged] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
		  ch->speedRampingState);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [speedRampingState] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
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

}

static void CCONV
PhidgetRCServo_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetRCServo));
}

API_PRETURN
PhidgetRCServo_create(PhidgetRCServoHandle *phidp) {

	CHANNELCREATE_BODY(RCServo, PHIDCHCLASS_RCSERVO);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_delete(PhidgetRCServoHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetRCServo_setDataInterval(PhidgetRCServoHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetRCServo_getDataInterval(PhidgetRCServoHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
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
PhidgetRCServo_getMinDataInterval(PhidgetRCServoHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
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
PhidgetRCServo_getMaxDataInterval(PhidgetRCServoHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_RCC1000_RCSERVO_100:
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
PhidgetRCServo_setEngaged(PhidgetRCServoHandle ch, int engaged) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENGAGED, NULL, NULL, "%d", engaged));
}

API_PRETURN
PhidgetRCServo_getEngaged(PhidgetRCServoHandle ch, int *engaged) {

	TESTPTRL(ch);
	TESTPTRL(engaged);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	*engaged = ch->engaged;
	if (ch->engaged == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getIsMoving(PhidgetRCServoHandle ch, int *isMoving) {

	TESTPTRL(ch);
	TESTPTRL(isMoving);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*isMoving = ch->isMoving;
	if (ch->isMoving == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinPosition(PhidgetRCServoHandle ch, double *minPosition) {

	TESTPTRL(ch);
	TESTPTRL(minPosition);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	*minPosition = ch->minPosition;
	if (ch->minPosition == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxPosition(PhidgetRCServoHandle ch, double *maxPosition) {

	TESTPTRL(ch);
	TESTPTRL(maxPosition);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	*maxPosition = ch->maxPosition;
	if (ch->maxPosition == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setMinPulseWidth(PhidgetRCServoHandle ch, double minPulseWidth) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETMINPULSEWIDTH, NULL, NULL, "%g",
	  minPulseWidth));
}

API_PRETURN
PhidgetRCServo_getMinPulseWidth(PhidgetRCServoHandle ch, double *minPulseWidth) {

	TESTPTRL(ch);
	TESTPTRL(minPulseWidth);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minPulseWidth = ch->minPulseWidth;
	if (ch->minPulseWidth == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setMaxPulseWidth(PhidgetRCServoHandle ch, double maxPulseWidth) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETMAXPULSEWIDTH, NULL, NULL, "%g",
	  maxPulseWidth));
}

API_PRETURN
PhidgetRCServo_getMaxPulseWidth(PhidgetRCServoHandle ch, double *maxPulseWidth) {

	TESTPTRL(ch);
	TESTPTRL(maxPulseWidth);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxPulseWidth = ch->maxPulseWidth;
	if (ch->maxPulseWidth == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinPulseWidthLimit(PhidgetRCServoHandle ch, double *minPulseWidthLimit) {

	TESTPTRL(ch);
	TESTPTRL(minPulseWidthLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minPulseWidthLimit = ch->minPulseWidthLimit;
	if (ch->minPulseWidthLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxPulseWidthLimit(PhidgetRCServoHandle ch, double *maxPulseWidthLimit) {

	TESTPTRL(ch);
	TESTPTRL(maxPulseWidthLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxPulseWidthLimit = ch->maxPulseWidthLimit;
	if (ch->maxPulseWidthLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setSpeedRampingState(PhidgetRCServoHandle ch, int speedRampingState) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSPEEDRAMPINGSTATE, NULL, NULL, "%d",
	  speedRampingState));
}

API_PRETURN
PhidgetRCServo_getSpeedRampingState(PhidgetRCServoHandle ch, int *speedRampingState) {

	TESTPTRL(ch);
	TESTPTRL(speedRampingState);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	*speedRampingState = ch->speedRampingState;
	if (ch->speedRampingState == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setTorque(PhidgetRCServoHandle ch, double torque) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g", torque));
}

API_PRETURN
PhidgetRCServo_getTorque(PhidgetRCServoHandle ch, double *torque) {

	TESTPTRL(ch);
	TESTPTRL(torque);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*torque = ch->torque;
	if (ch->torque == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMinTorque(PhidgetRCServoHandle ch, double *minTorque) {

	TESTPTRL(ch);
	TESTPTRL(minTorque);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minTorque = ch->minTorque;
	if (ch->minTorque == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_getMaxTorque(PhidgetRCServoHandle ch, double *maxTorque) {

	TESTPTRL(ch);
	TESTPTRL(maxTorque);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1000_RCSERVO_OLD1_200:
	case PHIDCHUID_1000_RCSERVO_OLD2_200:
	case PHIDCHUID_1000_RCSERVO_300:
	case PHIDCHUID_1000_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_OLD1_200:
	case PHIDCHUID_1001_RCSERVO_OLD2_200:
	case PHIDCHUID_1001_RCSERVO_313:
	case PHIDCHUID_1001_RCSERVO_400:
	case PHIDCHUID_1061_RCSERVO_100:
	case PHIDCHUID_1061_RCSERVO_200:
	case PHIDCHUID_1061_RCSERVO_300:
	case PHIDCHUID_1066_RCSERVO_100:
	case PHIDCHUID_RCC1000_RCSERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxTorque = ch->maxTorque;
	if (ch->maxTorque == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setVoltage(PhidgetRCServoHandle ch, PhidgetRCServo_Voltage voltage) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETVOLTAGE, NULL, NULL, "%d", voltage));
}

API_PRETURN
PhidgetRCServo_getVoltage(PhidgetRCServoHandle ch, PhidgetRCServo_Voltage *voltage) {

	TESTPTRL(ch);
	TESTPTRL(voltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_VINTSERVO_S2313M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S3317M_SERVO_100:
	case PHIDCHUID_VINTSERVO_S4309M_SERVO_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*voltage = ch->voltage;
	if (ch->voltage == (PhidgetRCServo_Voltage)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnPositionChangeHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);

	ch->PositionChange = fptr;
	ch->PositionChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnTargetPositionReachedHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnTargetPositionReachedCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);

	ch->TargetPositionReached = fptr;
	ch->TargetPositionReachedCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetRCServo_setOnVelocityChangeHandler(PhidgetRCServoHandle ch,
  PhidgetRCServo_OnVelocityChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RCSERVO);

	ch->VelocityChange = fptr;
	ch->VelocityChangeCtx = ctx;

	return (EPHIDGET_OK);
}
