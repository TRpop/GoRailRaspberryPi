/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/accelerometerdevice.h"
#include "device/spatialdevice.h"
static void CCONV PhidgetAccelerometer_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetAccelerometer_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetAccelerometer_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetAccelerometer_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetAccelerometer_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetAccelerometer_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetAccelerometer_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetAccelerometer {
	struct _PhidgetChannel phid;
	double acceleration[3];
	double minAcceleration[3];
	double maxAcceleration[3];
	double accelerationChangeTrigger;
	double minAccelerationChangeTrigger;
	double maxAccelerationChangeTrigger;
	int axisCount;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double timestamp;
	PhidgetAccelerometer_OnAccelerationChangeCallback AccelerationChange;
	void *AccelerationChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetAccelerometerHandle ch;
	int version;

	ch = (PhidgetAccelerometerHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		logerr("%"PRIphid": bad version %d != 1", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	memcpy(&ch->acceleration, getBridgePacketDoubleArrayByName(bp, "acceleration"),
	  sizeof (double) * 3);
	memcpy(&ch->minAcceleration, getBridgePacketDoubleArrayByName(bp, "minAcceleration"),
	  sizeof (double) * 3);
	memcpy(&ch->maxAcceleration, getBridgePacketDoubleArrayByName(bp, "maxAcceleration"),
	  sizeof (double) * 3);
	ch->accelerationChangeTrigger = getBridgePacketDoubleByName(bp, "accelerationChangeTrigger");
	ch->minAccelerationChangeTrigger = getBridgePacketDoubleByName(bp, "minAccelerationChangeTrigger");
	ch->maxAccelerationChangeTrigger = getBridgePacketDoubleByName(bp, "maxAccelerationChangeTrigger");
	ch->axisCount = getBridgePacketInt32ByName(bp, "axisCount");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->timestamp = getBridgePacketDoubleByName(bp, "timestamp");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetAccelerometerHandle ch;

	ch = (PhidgetAccelerometerHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",acceleration=%3G"
	  ",minAcceleration=%3G"
	  ",maxAcceleration=%3G"
	  ",accelerationChangeTrigger=%g"
	  ",minAccelerationChangeTrigger=%g"
	  ",maxAccelerationChangeTrigger=%g"
	  ",axisCount=%d"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",timestamp=%g"
	  ,1 /* class version */
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->accelerationChangeTrigger
	  ,ch->minAccelerationChangeTrigger
	  ,ch->maxAccelerationChangeTrigger
	  ,ch->axisCount
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->timestamp
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetAccelerometerHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetAccelerometerHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETCHANGETRIGGER:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minAccelerationChangeTrigger,
		  ch->maxAccelerationChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->accelerationChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "AccelerationChangeTrigger");
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
	case BP_ACCELERATIONCHANGE:
		memcpy(&ch->acceleration, getBridgePacketDoubleArray(bp, 0), sizeof (double) * 3);
		ch->timestamp = getBridgePacketDouble(bp, 1);
		FIRECH(ch, AccelerationChange, ch->acceleration, ch->timestamp);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetAccelerometerDeviceHandle parentAccelerometer;
	PhidgetSpatialDeviceHandle parentSpatial;
	PhidgetAccelerometerHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetAccelerometerHandle)phid;

	ret = EPHIDGET_OK;

	parentAccelerometer = (PhidgetAccelerometerDeviceHandle)phid->parent;
	parentSpatial = (PhidgetSpatialDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1008_ACCELEROMETER_000:
		ch->dataInterval = 273;
		ch->timestamp = parentAccelerometer->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 2;
		ch->maxAcceleration[1] = 2;
		ch->maxAcceleration[2] = 2;
		ch->maxAccelerationChangeTrigger = 4;
		ch->minDataInterval = 39;
		ch->minAcceleration[0] = -2;
		ch->minAcceleration[1] = -2;
		ch->minAcceleration[2] = -2;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentAccelerometer->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentAccelerometer->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentAccelerometer->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 2;
		break;
	case PHIDCHUID_1041_ACCELEROMETER_200:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 1;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1042_ACCELEROMETER_300:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 4;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1043_ACCELEROMETER_300:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 1;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1044_ACCELEROMETER_400:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 4;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1049_ACCELEROMETER_000:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 5;
		ch->maxAcceleration[1] = 5;
		ch->maxAcceleration[2] = 5;
		ch->maxAccelerationChangeTrigger = 10;
		ch->minDataInterval = 1;
		ch->minAcceleration[0] = -5;
		ch->minAcceleration[1] = -5;
		ch->minAcceleration[2] = -5;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1053_ACCELEROMETER_300:
		ch->dataInterval = 256;
		ch->timestamp = parentAccelerometer->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 5;
		ch->maxAcceleration[1] = 5;
		ch->maxAcceleration[2] = 5;
		ch->maxAccelerationChangeTrigger = 10;
		ch->minDataInterval = 16;
		ch->minAcceleration[0] = -5;
		ch->minAcceleration[1] = -5;
		ch->minAcceleration[2] = -5;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentAccelerometer->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentAccelerometer->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentAccelerometer->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 2;
		break;
	case PHIDCHUID_1056_ACCELEROMETER_000:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 5;
		ch->maxAcceleration[1] = 5;
		ch->maxAcceleration[2] = 5;
		ch->maxAccelerationChangeTrigger = 10;
		ch->minDataInterval = 4;
		ch->minAcceleration[0] = -5;
		ch->minAcceleration[1] = -5;
		ch->minAcceleration[2] = -5;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1056_ACCELEROMETER_200:
		ch->dataInterval = 256;
		ch->timestamp = parentSpatial->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 5;
		ch->maxAcceleration[1] = 5;
		ch->maxAcceleration[2] = 5;
		ch->maxAccelerationChangeTrigger = 10;
		ch->minDataInterval = 4;
		ch->minAcceleration[0] = -5;
		ch->minAcceleration[1] = -5;
		ch->minAcceleration[2] = -5;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentSpatial->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentSpatial->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentSpatial->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_1059_ACCELEROMETER_400:
		ch->dataInterval = 256;
		ch->timestamp = parentAccelerometer->timestamp[ch->phid.index];
		ch->maxDataInterval = 1000;
		ch->maxAcceleration[0] = 3;
		ch->maxAcceleration[1] = 3;
		ch->maxAcceleration[2] = 3;
		ch->maxAccelerationChangeTrigger = 6;
		ch->minDataInterval = 16;
		ch->minAcceleration[0] = -3;
		ch->minAcceleration[1] = -3;
		ch->minAcceleration[2] = -3;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = parentAccelerometer->acceleration[ch->phid.index][0];
		ch->acceleration[1] = parentAccelerometer->acceleration[ch->phid.index][1];
		ch->acceleration[2] = parentAccelerometer->acceleration[ch->phid.index][2];
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_MOT1100_ACCELEROMETER_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 20;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = PUNK_DBL;
		ch->acceleration[1] = PUNK_DBL;
		ch->acceleration[2] = PUNK_DBL;
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 10;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = PUNK_DBL;
		ch->acceleration[1] = PUNK_DBL;
		ch->acceleration[2] = PUNK_DBL;
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		ch->dataInterval = 250;
		ch->timestamp = PUNK_DBL;
		ch->maxDataInterval = 60000;
		ch->maxAcceleration[0] = 8;
		ch->maxAcceleration[1] = 8;
		ch->maxAcceleration[2] = 8;
		ch->maxAccelerationChangeTrigger = 16;
		ch->minDataInterval = 20;
		ch->minAcceleration[0] = -8;
		ch->minAcceleration[1] = -8;
		ch->minAcceleration[2] = -8;
		ch->minAccelerationChangeTrigger = 0;
		ch->acceleration[0] = PUNK_DBL;
		ch->acceleration[1] = PUNK_DBL;
		ch->acceleration[2] = PUNK_DBL;
		ch->accelerationChangeTrigger = 0;
		ch->axisCount = 3;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetAccelerometerHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetAccelerometerHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1008_ACCELEROMETER_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1041_ACCELEROMETER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1042_ACCELEROMETER_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1043_ACCELEROMETER_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1044_ACCELEROMETER_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1049_ACCELEROMETER_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1053_ACCELEROMETER_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1056_ACCELEROMETER_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1056_ACCELEROMETER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1059_ACCELEROMETER_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_MOT1100_ACCELEROMETER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_MOT1101_ACCELEROMETER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->accelerationChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [accelerationChangeTrigger] default: %d", phid, ret);
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
	PhidgetAccelerometerHandle ch;

	ch = (PhidgetAccelerometerHandle)phid;

	if(ch->acceleration[0] != PUNK_DBL &&
	  ch->acceleration[1] != PUNK_DBL &&
	  ch->acceleration[2] != PUNK_DBL &&
	  ch->timestamp != PUNK_DBL)
		FIRECH(ch, AccelerationChange, ch->acceleration, ch->timestamp);

}

static void CCONV
PhidgetAccelerometer_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetAccelerometer));
}

API_PRETURN
PhidgetAccelerometer_create(PhidgetAccelerometerHandle *phidp) {

	CHANNELCREATE_BODY(Accelerometer, PHIDCHCLASS_ACCELEROMETER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_delete(PhidgetAccelerometerHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetAccelerometer_getAcceleration(PhidgetAccelerometerHandle ch, double (*acceleration)[3]) {

	TESTPTRL(ch);
	TESTPTRL(acceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	(*acceleration)[0] = ch->acceleration[0];
	if (ch->acceleration[0] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*acceleration)[1] = ch->acceleration[1];
	if (ch->acceleration[1] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*acceleration)[2] = ch->acceleration[2];
	if (ch->acceleration[2] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMinAcceleration(PhidgetAccelerometerHandle ch, double (*minAcceleration)[3]) {

	TESTPTRL(ch);
	TESTPTRL(minAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	(*minAcceleration)[0] = ch->minAcceleration[0];
	if (ch->minAcceleration[0] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*minAcceleration)[1] = ch->minAcceleration[1];
	if (ch->minAcceleration[1] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*minAcceleration)[2] = ch->minAcceleration[2];
	if (ch->minAcceleration[2] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMaxAcceleration(PhidgetAccelerometerHandle ch, double (*maxAcceleration)[3]) {

	TESTPTRL(ch);
	TESTPTRL(maxAcceleration);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	(*maxAcceleration)[0] = ch->maxAcceleration[0];
	if (ch->maxAcceleration[0] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*maxAcceleration)[1] = ch->maxAcceleration[1];
	if (ch->maxAcceleration[1] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	(*maxAcceleration)[2] = ch->maxAcceleration[2];
	if (ch->maxAcceleration[2] == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_setAccelerationChangeTrigger(PhidgetAccelerometerHandle ch,
  double accelerationChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  accelerationChangeTrigger));
}

API_PRETURN
PhidgetAccelerometer_getAccelerationChangeTrigger(PhidgetAccelerometerHandle ch,
  double *accelerationChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(accelerationChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*accelerationChangeTrigger = ch->accelerationChangeTrigger;
	if (ch->accelerationChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMinAccelerationChangeTrigger(PhidgetAccelerometerHandle ch,
  double *minAccelerationChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minAccelerationChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*minAccelerationChangeTrigger = ch->minAccelerationChangeTrigger;
	if (ch->minAccelerationChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMaxAccelerationChangeTrigger(PhidgetAccelerometerHandle ch,
  double *maxAccelerationChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxAccelerationChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*maxAccelerationChangeTrigger = ch->maxAccelerationChangeTrigger;
	if (ch->maxAccelerationChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getAxisCount(PhidgetAccelerometerHandle ch, int *axisCount) {

	TESTPTRL(ch);
	TESTPTRL(axisCount);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*axisCount = ch->axisCount;
	if (ch->axisCount == (int)PUNK_INT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_setDataInterval(PhidgetAccelerometerHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetAccelerometer_getDataInterval(PhidgetAccelerometerHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMinDataInterval(PhidgetAccelerometerHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getMaxDataInterval(PhidgetAccelerometerHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_getTimestamp(PhidgetAccelerometerHandle ch, double *timestamp) {

	TESTPTRL(ch);
	TESTPTRL(timestamp);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_MOT1100_ACCELEROMETER_100:
	case PHIDCHUID_MOT1100_ACCELEROMETER_200:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*timestamp = ch->timestamp;
	if (ch->timestamp == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetAccelerometer_setOnAccelerationChangeHandler(PhidgetAccelerometerHandle ch,
  PhidgetAccelerometer_OnAccelerationChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_ACCELEROMETER);

	ch->AccelerationChange = fptr;
	ch->AccelerationChangeCtx = ctx;

	return (EPHIDGET_OK);
}
