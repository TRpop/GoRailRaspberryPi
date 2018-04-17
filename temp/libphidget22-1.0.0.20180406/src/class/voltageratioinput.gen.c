/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/interfacekitdevice.h"
#include "device/bridgedevice.h"
#include "device/motorcontroldevice.h"
static void CCONV PhidgetVoltageRatioInput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetVoltageRatioInput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetVoltageRatioInput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetVoltageRatioInput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetVoltageRatioInput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetVoltageRatioInput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetVoltageRatioInput_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetVoltageRatioInput {
	struct _PhidgetChannel phid;
	int bridgeEnabled;
	PhidgetVoltageRatioInput_BridgeGain bridgeGain;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	PhidgetVoltageRatioInput_SensorType sensorType;
	Phidget_UnitInfo sensorUnit;
	uint8_t sensorUnitValid;
	double sensorValue;
	double sensorValueChangeTrigger;
	double voltageRatio;
	double minVoltageRatio;
	double maxVoltageRatio;
	double voltageRatioChangeTrigger;
	double minVoltageRatioChangeTrigger;
	double maxVoltageRatioChangeTrigger;
	PhidgetVoltageRatioInput_OnSensorChangeCallback SensorChange;
	void *SensorChangeCtx;
	PhidgetVoltageRatioInput_OnVoltageRatioChangeCallback VoltageRatioChange;
	void *VoltageRatioChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageRatioInputHandle ch;
	int version;

	ch = (PhidgetVoltageRatioInputHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->bridgeEnabled = getBridgePacketInt32ByName(bp, "bridgeEnabled");
	ch->bridgeGain = getBridgePacketInt32ByName(bp, "bridgeGain");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->sensorType = getBridgePacketInt32ByName(bp, "sensorType");
	ch->sensorValue = getBridgePacketDoubleByName(bp, "sensorValue");
	ch->sensorValueChangeTrigger = getBridgePacketDoubleByName(bp, "sensorValueChangeTrigger");
	ch->voltageRatio = getBridgePacketDoubleByName(bp, "voltageRatio");
	ch->minVoltageRatio = getBridgePacketDoubleByName(bp, "minVoltageRatio");
	ch->maxVoltageRatio = getBridgePacketDoubleByName(bp, "maxVoltageRatio");
	ch->voltageRatioChangeTrigger = getBridgePacketDoubleByName(bp, "voltageRatioChangeTrigger");
	ch->minVoltageRatioChangeTrigger = getBridgePacketDoubleByName(bp, "minVoltageRatioChangeTrigger");
	ch->maxVoltageRatioChangeTrigger = getBridgePacketDoubleByName(bp, "maxVoltageRatioChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetVoltageRatioInputHandle ch;

	ch = (PhidgetVoltageRatioInputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",bridgeEnabled=%d"
	  ",bridgeGain=%d"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",sensorType=%d"
	  ",sensorValue=%g"
	  ",sensorValueChangeTrigger=%g"
	  ",voltageRatio=%g"
	  ",minVoltageRatio=%g"
	  ",maxVoltageRatio=%g"
	  ",voltageRatioChangeTrigger=%g"
	  ",minVoltageRatioChangeTrigger=%g"
	  ",maxVoltageRatioChangeTrigger=%g"
	  ,0 /* class version */
	  ,ch->bridgeEnabled
	  ,ch->bridgeGain
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->sensorType
	  ,ch->sensorValue
	  ,ch->sensorValueChangeTrigger
	  ,ch->voltageRatio
	  ,ch->minVoltageRatio
	  ,ch->maxVoltageRatio
	  ,ch->voltageRatioChangeTrigger
	  ,ch->minVoltageRatioChangeTrigger
	  ,ch->maxVoltageRatioChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageRatioInputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetVoltageRatioInputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETENABLED:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->bridgeEnabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "BridgeEnabled");
		break;
	case BP_SETBRIDGEGAIN:
		if (!supportedBridgeGain(phid, (PhidgetVoltageRatioInput_BridgeGain)getBridgePacketInt32(bp,
		  0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->bridgeGain = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "BridgeGain");
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
	case BP_SETSENSORTYPE:
		if (!supportedVoltageRatioSensorType(phid,
		  (PhidgetVoltageRatioInput_SensorType)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->sensorType = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "SensorType");
		break;
	case BP_SETSENSORVALUECHANGETRIGGER:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->sensorValueChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "SensorValueChangeTrigger");
		break;
	case BP_SETCHANGETRIGGER:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minVoltageRatioChangeTrigger,
		  ch->maxVoltageRatioChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->voltageRatioChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "VoltageRatioChangeTrigger");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetInterfaceKitDeviceHandle parentInterfaceKit;
	PhidgetMotorControlDeviceHandle parentMotorControl;
	PhidgetBridgeDeviceHandle parentBridge;
	PhidgetVoltageRatioInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetVoltageRatioInputHandle)phid;

	ret = EPHIDGET_OK;

	parentInterfaceKit = (PhidgetInterfaceKitDeviceHandle)phid->parent;
	parentMotorControl = (PhidgetMotorControlDeviceHandle)phid->parent;
	parentBridge = (PhidgetBridgeDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		ch->bridgeEnabled = 1;
		ch->bridgeGain = BRIDGE_GAIN_128;
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 1000;
		ch->voltageRatio = parentBridge->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = -1;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		break;
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		ch->bridgeEnabled = 1;
		ch->bridgeGain = BRIDGE_GAIN_128;
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 1000;
		ch->voltageRatio = parentBridge->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = -1;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		break;
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentMotorControl->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = parentInterfaceKit->voltageRatio[ch->phid.index];
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		ch->bridgeEnabled = 1;
		ch->bridgeGain = BRIDGE_GAIN_128;
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = -0.5;
		ch->maxVoltageRatio = 0.5;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 0.5;
		break;
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = 0;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 1;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGERATIO;
		ch->voltageRatio = PUNK_DBL;
		ch->minVoltageRatio = -1;
		ch->maxVoltageRatio = 1;
		ch->voltageRatioChangeTrigger = 0;
		ch->minVoltageRatioChangeTrigger = 0;
		ch->maxVoltageRatioChangeTrigger = 2;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetVoltageRatioInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetVoltageRatioInputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->bridgeEnabled);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeEnabled] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETBRIDGEGAIN, NULL, NULL, "%d", ch->bridgeGain);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeGain] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->bridgeEnabled);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeEnabled] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETBRIDGEGAIN, NULL, NULL, "%d", ch->bridgeGain);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeGain] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->bridgeEnabled);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeEnabled] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETBRIDGEGAIN, NULL, NULL, "%d", ch->bridgeGain);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [bridgeGain] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
		ret = bridgeSendToDevice(phid, BP_SETSENSORTYPE, NULL, NULL, "%d", ch->sensorType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageRatioChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRatioChangeTrigger] default: %d", phid, ret);
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
PhidgetVoltageRatioInput_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetVoltageRatioInput));
}

API_PRETURN
PhidgetVoltageRatioInput_create(PhidgetVoltageRatioInputHandle *phidp) {

	CHANNELCREATE_BODY(VoltageRatioInput, PHIDCHCLASS_VOLTAGERATIOINPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_delete(PhidgetVoltageRatioInputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetVoltageRatioInput_setBridgeEnabled(PhidgetVoltageRatioInputHandle ch, int bridgeEnabled) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLED, NULL, NULL, "%d",
	  bridgeEnabled));
}

API_PRETURN
PhidgetVoltageRatioInput_getBridgeEnabled(PhidgetVoltageRatioInputHandle ch, int *bridgeEnabled) {

	TESTPTRL(ch);
	TESTPTRL(bridgeEnabled);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*bridgeEnabled = ch->bridgeEnabled;
	if (ch->bridgeEnabled == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setBridgeGain(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_BridgeGain bridgeGain) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBRIDGEGAIN, NULL, NULL, "%d",
	  bridgeGain));
}

API_PRETURN
PhidgetVoltageRatioInput_getBridgeGain(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_BridgeGain *bridgeGain) {

	TESTPTRL(ch);
	TESTPTRL(bridgeGain);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1011_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1013_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_821:
	case PHIDCHUID_1018_VOLTAGERATIOINPUT_900:
	case PHIDCHUID_1065_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_000:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_120:
	case PHIDCHUID_1202_VOLTAGERATIOINPUT_300:
	case PHIDCHUID_HUB_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_DCC1000_VOLTAGERATIOINPUT_200:
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*bridgeGain = ch->bridgeGain;
	if (ch->bridgeGain == (PhidgetVoltageRatioInput_BridgeGain)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setDataInterval(PhidgetVoltageRatioInputHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetVoltageRatioInput_getDataInterval(PhidgetVoltageRatioInputHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMinDataInterval(PhidgetVoltageRatioInputHandle ch,
  uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMaxDataInterval(PhidgetVoltageRatioInputHandle ch,
  uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setSensorType(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_SensorType sensorType) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSENSORTYPE, NULL, NULL, "%d",
	  sensorType));
}

API_PRETURN
PhidgetVoltageRatioInput_getSensorType(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_SensorType *sensorType) {

	TESTPTRL(ch);
	TESTPTRL(sensorType);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*sensorType = ch->sensorType;
	if (ch->sensorType == (PhidgetVoltageRatioInput_SensorType)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getSensorUnit(PhidgetVoltageRatioInputHandle ch,
  Phidget_UnitInfo *sensorUnit) {

	TESTPTRL(ch);
	TESTPTRL(sensorUnit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*sensorUnit = ch->sensorUnit;
	if (ch->sensorUnitValid == PFALSE)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getSensorValue(PhidgetVoltageRatioInputHandle ch, double *sensorValue) {

	TESTPTRL(ch);
	TESTPTRL(sensorValue);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*sensorValue = ch->sensorValue;
	if (ch->sensorValue == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setSensorValueChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double sensorValueChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL,
	  "%g", sensorValueChangeTrigger));
}

API_PRETURN
PhidgetVoltageRatioInput_getSensorValueChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double *sensorValueChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(sensorValueChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_1046_VOLTAGERATIOINPUT_102:
	case PHIDCHUID_DAQ1500_VOLTAGERATIOINPUT_100:
	case PHIDCHUID_HIN1100_VOLTAGERATIOINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*sensorValueChangeTrigger = ch->sensorValueChangeTrigger;
	if (ch->sensorValueChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getVoltageRatio(PhidgetVoltageRatioInputHandle ch, double *voltageRatio) {

	TESTPTRL(ch);
	TESTPTRL(voltageRatio);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*voltageRatio = ch->voltageRatio;
	if (ch->voltageRatio == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMinVoltageRatio(PhidgetVoltageRatioInputHandle ch,
  double *minVoltageRatio) {

	TESTPTRL(ch);
	TESTPTRL(minVoltageRatio);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*minVoltageRatio = ch->minVoltageRatio;
	if (ch->minVoltageRatio == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMaxVoltageRatio(PhidgetVoltageRatioInputHandle ch,
  double *maxVoltageRatio) {

	TESTPTRL(ch);
	TESTPTRL(maxVoltageRatio);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*maxVoltageRatio = ch->maxVoltageRatio;
	if (ch->maxVoltageRatio == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setVoltageRatioChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double voltageRatioChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  voltageRatioChangeTrigger));
}

API_PRETURN
PhidgetVoltageRatioInput_getVoltageRatioChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double *voltageRatioChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(voltageRatioChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*voltageRatioChangeTrigger = ch->voltageRatioChangeTrigger;
	if (ch->voltageRatioChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMinVoltageRatioChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double *minVoltageRatioChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minVoltageRatioChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*minVoltageRatioChangeTrigger = ch->minVoltageRatioChangeTrigger;
	if (ch->minVoltageRatioChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_getMaxVoltageRatioChangeTrigger(PhidgetVoltageRatioInputHandle ch,
  double *maxVoltageRatioChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxVoltageRatioChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);
	TESTATTACHEDL(ch);

	*maxVoltageRatioChangeTrigger = ch->maxVoltageRatioChangeTrigger;
	if (ch->maxVoltageRatioChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setOnSensorChangeHandler(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_OnSensorChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);

	ch->SensorChange = fptr;
	ch->SensorChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageRatioInput_setOnVoltageRatioChangeHandler(PhidgetVoltageRatioInputHandle ch,
  PhidgetVoltageRatioInput_OnVoltageRatioChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGERATIOINPUT);

	ch->VoltageRatioChange = fptr;
	ch->VoltageRatioChangeCtx = ctx;

	return (EPHIDGET_OK);
}
