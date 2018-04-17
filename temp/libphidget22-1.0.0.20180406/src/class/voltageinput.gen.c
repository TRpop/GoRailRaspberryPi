/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/interfacekitdevice.h"
#include "device/temperaturesensordevice.h"
#include "device/phsensordevice.h"
#include "device/motorcontroldevice.h"
static void CCONV PhidgetVoltageInput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetVoltageInput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetVoltageInput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetVoltageInput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetVoltageInput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetVoltageInput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetVoltageInput_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetVoltageInput {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	Phidget_PowerSupply powerSupply;
	PhidgetVoltageInput_SensorType sensorType;
	Phidget_UnitInfo sensorUnit;
	uint8_t sensorUnitValid;
	double sensorValue;
	double sensorValueChangeTrigger;
	double voltage;
	double minVoltage;
	double maxVoltage;
	double voltageChangeTrigger;
	double minVoltageChangeTrigger;
	double maxVoltageChangeTrigger;
	PhidgetVoltageInput_VoltageRange voltageRange;
	PhidgetVoltageInput_OnSensorChangeCallback SensorChange;
	void *SensorChangeCtx;
	PhidgetVoltageInput_OnVoltageChangeCallback VoltageChange;
	void *VoltageChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageInputHandle ch;
	int version;

	ch = (PhidgetVoltageInputHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->powerSupply = getBridgePacketInt32ByName(bp, "powerSupply");
	ch->sensorType = getBridgePacketInt32ByName(bp, "sensorType");
	ch->sensorValue = getBridgePacketDoubleByName(bp, "sensorValue");
	ch->sensorValueChangeTrigger = getBridgePacketDoubleByName(bp, "sensorValueChangeTrigger");
	ch->voltage = getBridgePacketDoubleByName(bp, "voltage");
	ch->minVoltage = getBridgePacketDoubleByName(bp, "minVoltage");
	ch->maxVoltage = getBridgePacketDoubleByName(bp, "maxVoltage");
	ch->voltageChangeTrigger = getBridgePacketDoubleByName(bp, "voltageChangeTrigger");
	ch->minVoltageChangeTrigger = getBridgePacketDoubleByName(bp, "minVoltageChangeTrigger");
	ch->maxVoltageChangeTrigger = getBridgePacketDoubleByName(bp, "maxVoltageChangeTrigger");
	ch->voltageRange = getBridgePacketInt32ByName(bp, "voltageRange");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetVoltageInputHandle ch;

	ch = (PhidgetVoltageInputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",powerSupply=%d"
	  ",sensorType=%d"
	  ",sensorValue=%g"
	  ",sensorValueChangeTrigger=%g"
	  ",voltage=%g"
	  ",minVoltage=%g"
	  ",maxVoltage=%g"
	  ",voltageChangeTrigger=%g"
	  ",minVoltageChangeTrigger=%g"
	  ",maxVoltageChangeTrigger=%g"
	  ",voltageRange=%d"
	  ,0 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->powerSupply
	  ,ch->sensorType
	  ,ch->sensorValue
	  ,ch->sensorValueChangeTrigger
	  ,ch->voltage
	  ,ch->minVoltage
	  ,ch->maxVoltage
	  ,ch->voltageChangeTrigger
	  ,ch->minVoltageChangeTrigger
	  ,ch->maxVoltageChangeTrigger
	  ,ch->voltageRange
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetVoltageInputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetVoltageInputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
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
	case BP_SETPOWERSUPPLY:
		if (!supportedPowerSupply(phid, (Phidget_PowerSupply)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->powerSupply = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "PowerSupply");
		break;
	case BP_SETSENSORTYPE:
		if (!supportedVoltageSensorType(phid, (PhidgetVoltageInput_SensorType)getBridgePacketInt32(bp,
		  0)))
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
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minVoltageChangeTrigger,
		  ch->maxVoltageChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->voltageChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "VoltageChangeTrigger");
		break;
	case BP_SETVOLTAGERANGE:
		if (!supportedVoltageRange(phid, (PhidgetVoltageInput_VoltageRange)getBridgePacketInt32(bp,
		  0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->voltageRange = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "VoltageRange");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetTemperatureSensorDeviceHandle parentTemperatureSensor;
	PhidgetInterfaceKitDeviceHandle parentInterfaceKit;
	PhidgetMotorControlDeviceHandle parentMotorControl;
	PhidgetPHSensorDeviceHandle parentPHSensor;
	PhidgetVoltageInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetVoltageInputHandle)phid;

	ret = EPHIDGET_OK;

	parentTemperatureSensor = (PhidgetTemperatureSensorDeviceHandle)phid->parent;
	parentInterfaceKit = (PhidgetInterfaceKitDeviceHandle)phid->parent;
	parentMotorControl = (PhidgetMotorControlDeviceHandle)phid->parent;
	parentPHSensor = (PhidgetPHSensorDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->voltage = parentTemperatureSensor->voltage[ch->phid.index];
		ch->minVoltage = -0.01001;
		ch->maxVoltage = 0.07741;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.08742;
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->voltage = parentTemperatureSensor->voltage[ch->phid.index];
		ch->minVoltage = -0.01001;
		ch->maxVoltage = 0.07741;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.08742;
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->voltage = parentTemperatureSensor->voltage[ch->phid.index];
		ch->minVoltage = -0.01001;
		ch->maxVoltage = 0.07741;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.08742;
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->voltage = parentTemperatureSensor->voltage[ch->phid.index];
		ch->minVoltage = -0.01001;
		ch->maxVoltage = 0.07741;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.08742;
		break;
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 80;
		ch->maxDataInterval = 60000;
		ch->voltage = parentPHSensor->voltage[ch->phid.index];
		ch->minVoltage = -0.52711;
		ch->maxVoltage = 0.52687;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.1054;
		break;
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentMotorControl->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		ch->dataInterval = 256;
		ch->minDataInterval = 8;
		ch->maxDataInterval = 60000;
		ch->voltage = parentMotorControl->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 80;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 80;
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
		ch->dataInterval = 256;
		ch->minDataInterval = 16;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
		ch->dataInterval = 256;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 1000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = parentInterfaceKit->voltage[ch->phid.index];
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 1;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 50;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = -2;
		ch->maxVoltage = 2;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 4;
		ch->voltageRange = VOLTAGE_RANGE_400mV;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->sensorType = SENSOR_TYPE_VOLTAGE;
		ch->voltage = PUNK_DBL;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		ch->sensorValueChangeTrigger = 0;
		break;
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->powerSupply = POWER_SUPPLY_12V;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = 0;
		ch->maxVoltage = 5;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 5;
		break;
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = 8;
		ch->maxVoltage = 40;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 32;
		break;
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = -0.011;
		ch->maxVoltage = 0.079;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.09;
		break;
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = -0.011;
		ch->maxVoltage = 0.079;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.09;
		break;
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = -0.00043;
		ch->maxVoltage = 0.00318;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 0.09;
		break;
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = PUNK_DBL;
		ch->maxVoltage = PUNK_DBL;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 80;
		ch->voltageRange = VOLTAGE_RANGE_40V;
		break;
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 40;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = PUNK_DBL;
		ch->maxVoltage = PUNK_DBL;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 80;
		ch->voltageRange = VOLTAGE_RANGE_AUTO;
		break;
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 40;
		ch->maxDataInterval = 60000;
		ch->voltage = PUNK_DBL;
		ch->minVoltage = PUNK_DBL;
		ch->maxVoltage = PUNK_DBL;
		ch->voltageChangeTrigger = 0;
		ch->minVoltageChangeTrigger = 0;
		ch->maxVoltageChangeTrigger = 2;
		ch->voltageRange = VOLTAGE_RANGE_AUTO;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetVoltageInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetVoltageInputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVOLTAGERANGE, NULL, NULL, "%d", ch->voltageRange);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRange] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
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
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL, "%g",
		  ch->sensorValueChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [sensorValueChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETPOWERSUPPLY, NULL, NULL, "%d", ch->powerSupply);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [powerSupply] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVOLTAGERANGE, NULL, NULL, "%d", ch->voltageRange);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRange] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVOLTAGERANGE, NULL, NULL, "%d", ch->voltageRange);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRange] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->voltageChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETVOLTAGERANGE, NULL, NULL, "%d", ch->voltageRange);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [voltageRange] default: %d", phid, ret);
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
PhidgetVoltageInput_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetVoltageInput));
}

API_PRETURN
PhidgetVoltageInput_create(PhidgetVoltageInputHandle *phidp) {

	CHANNELCREATE_BODY(VoltageInput, PHIDCHCLASS_VOLTAGEINPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_delete(PhidgetVoltageInputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetVoltageInput_setDataInterval(PhidgetVoltageInputHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetVoltageInput_getDataInterval(PhidgetVoltageInputHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMinDataInterval(PhidgetVoltageInputHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMaxDataInterval(PhidgetVoltageInputHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setPowerSupply(PhidgetVoltageInputHandle ch, Phidget_PowerSupply powerSupply) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPOWERSUPPLY, NULL, NULL, "%d",
	  powerSupply));
}

API_PRETURN
PhidgetVoltageInput_getPowerSupply(PhidgetVoltageInputHandle ch, Phidget_PowerSupply *powerSupply) {

	TESTPTRL(ch);
	TESTPTRL(powerSupply);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
	case PHIDCHUID_ADP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*powerSupply = ch->powerSupply;
	if (ch->powerSupply == (Phidget_PowerSupply)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setSensorType(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_SensorType sensorType) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSENSORTYPE, NULL, NULL, "%d",
	  sensorType));
}

API_PRETURN
PhidgetVoltageInput_getSensorType(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_SensorType *sensorType) {

	TESTPTRL(ch);
	TESTPTRL(sensorType);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*sensorType = ch->sensorType;
	if (ch->sensorType == (PhidgetVoltageInput_SensorType)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getSensorUnit(PhidgetVoltageInputHandle ch, Phidget_UnitInfo *sensorUnit) {

	TESTPTRL(ch);
	TESTPTRL(sensorUnit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
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
PhidgetVoltageInput_getSensorValue(PhidgetVoltageInputHandle ch, double *sensorValue) {

	TESTPTRL(ch);
	TESTPTRL(sensorValue);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
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
PhidgetVoltageInput_setSensorValueChangeTrigger(PhidgetVoltageInputHandle ch,
  double sensorValueChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSENSORVALUECHANGETRIGGER, NULL, NULL,
	  "%g", sensorValueChangeTrigger));
}

API_PRETURN
PhidgetVoltageInput_getSensorValueChangeTrigger(PhidgetVoltageInputHandle ch,
  double *sensorValueChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(sensorValueChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1000_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1001_VOLTAGEINPUT_100:
	case PHIDCHUID_VCP1002_VOLTAGEINPUT_100:
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
PhidgetVoltageInput_getVoltage(PhidgetVoltageInputHandle ch, double *voltage) {

	TESTPTRL(ch);
	TESTPTRL(voltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*voltage = ch->voltage;
	if (ch->voltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMinVoltage(PhidgetVoltageInputHandle ch, double *minVoltage) {

	TESTPTRL(ch);
	TESTPTRL(minVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minVoltage = ch->minVoltage;
	if (ch->minVoltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMaxVoltage(PhidgetVoltageInputHandle ch, double *maxVoltage) {

	TESTPTRL(ch);
	TESTPTRL(maxVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*maxVoltage = ch->maxVoltage;
	if (ch->maxVoltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setVoltageChangeTrigger(PhidgetVoltageInputHandle ch, double voltageChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  voltageChangeTrigger));
}

API_PRETURN
PhidgetVoltageInput_getVoltageChangeTrigger(PhidgetVoltageInputHandle ch,
  double *voltageChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(voltageChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*voltageChangeTrigger = ch->voltageChangeTrigger;
	if (ch->voltageChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMinVoltageChangeTrigger(PhidgetVoltageInputHandle ch,
  double *minVoltageChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minVoltageChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*minVoltageChangeTrigger = ch->minVoltageChangeTrigger;
	if (ch->minVoltageChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_getMaxVoltageChangeTrigger(PhidgetVoltageInputHandle ch,
  double *maxVoltageChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxVoltageChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	*maxVoltageChangeTrigger = ch->maxVoltageChangeTrigger;
	if (ch->maxVoltageChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setVoltageRange(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_VoltageRange voltageRange) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETVOLTAGERANGE, NULL, NULL, "%d",
	  voltageRange));
}

API_PRETURN
PhidgetVoltageInput_getVoltageRange(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_VoltageRange *voltageRange) {

	TESTPTRL(ch);
	TESTPTRL(voltageRange);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1011_VOLTAGEINPUT_000:
	case PHIDCHUID_1013_VOLTAGEINPUT_000:
	case PHIDCHUID_1018_VOLTAGEINPUT_821:
	case PHIDCHUID_1018_VOLTAGEINPUT_900:
	case PHIDCHUID_1048_VOLTAGEINPUT_100:
	case PHIDCHUID_1051_VOLTAGEINPUT_200:
	case PHIDCHUID_1051_VOLTAGEINPUT_300:
	case PHIDCHUID_1051_VOLTAGEINPUT_400:
	case PHIDCHUID_1058_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_100:
	case PHIDCHUID_1065_VOLTAGEINPUT_SUPPLY_100:
	case PHIDCHUID_1202_VOLTAGEINPUT_000:
	case PHIDCHUID_1202_VOLTAGEINPUT_120:
	case PHIDCHUID_1202_VOLTAGEINPUT_300:
	case PHIDCHUID_HUB_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1000_VOLTAGEINPUT_100:
	case PHIDCHUID_DAQ1400_VOLTAGEINPUT_100:
	case PHIDCHUID_SAF1000_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1100_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1101_VOLTAGEINPUT_100:
	case PHIDCHUID_TMP1300_VOLTAGEINPUT_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*voltageRange = ch->voltageRange;
	if (ch->voltageRange == (PhidgetVoltageInput_VoltageRange)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setOnSensorChangeHandler(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_OnSensorChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);

	ch->SensorChange = fptr;
	ch->SensorChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetVoltageInput_setOnVoltageChangeHandler(PhidgetVoltageInputHandle ch,
  PhidgetVoltageInput_OnVoltageChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_VOLTAGEINPUT);

	ch->VoltageChange = fptr;
	ch->VoltageChangeCtx = ctx;

	return (EPHIDGET_OK);
}
