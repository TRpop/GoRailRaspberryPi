/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/temperaturesensordevice.h"
static void CCONV PhidgetTemperatureSensor_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetTemperatureSensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetTemperatureSensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetTemperatureSensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetTemperatureSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetTemperatureSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetTemperatureSensor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetTemperatureSensor {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	PhidgetTemperatureSensor_RTDType RTDType;
	Phidget_RTDWireSetup RTDWireSetup;
	double temperature;
	double minTemperature;
	double maxTemperature;
	double temperatureChangeTrigger;
	double minTemperatureChangeTrigger;
	double maxTemperatureChangeTrigger;
	PhidgetTemperatureSensor_ThermocoupleType thermocoupleType;
	PhidgetTemperatureSensor_OnTemperatureChangeCallback TemperatureChange;
	void *TemperatureChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetTemperatureSensorHandle ch;
	int version;

	ch = (PhidgetTemperatureSensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->RTDType = getBridgePacketInt32ByName(bp, "RTDType");
	ch->RTDWireSetup = getBridgePacketInt32ByName(bp, "RTDWireSetup");
	ch->temperature = getBridgePacketDoubleByName(bp, "temperature");
	ch->minTemperature = getBridgePacketDoubleByName(bp, "minTemperature");
	ch->maxTemperature = getBridgePacketDoubleByName(bp, "maxTemperature");
	ch->temperatureChangeTrigger = getBridgePacketDoubleByName(bp, "temperatureChangeTrigger");
	ch->minTemperatureChangeTrigger = getBridgePacketDoubleByName(bp, "minTemperatureChangeTrigger");
	ch->maxTemperatureChangeTrigger = getBridgePacketDoubleByName(bp, "maxTemperatureChangeTrigger");
	ch->thermocoupleType = getBridgePacketInt32ByName(bp, "thermocoupleType");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetTemperatureSensorHandle ch;

	ch = (PhidgetTemperatureSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",RTDType=%d"
	  ",RTDWireSetup=%d"
	  ",temperature=%g"
	  ",minTemperature=%g"
	  ",maxTemperature=%g"
	  ",temperatureChangeTrigger=%g"
	  ",minTemperatureChangeTrigger=%g"
	  ",maxTemperatureChangeTrigger=%g"
	  ",thermocoupleType=%d"
	  ,0 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->RTDType
	  ,ch->RTDWireSetup
	  ,ch->temperature
	  ,ch->minTemperature
	  ,ch->maxTemperature
	  ,ch->temperatureChangeTrigger
	  ,ch->minTemperatureChangeTrigger
	  ,ch->maxTemperatureChangeTrigger
	  ,ch->thermocoupleType
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetTemperatureSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetTemperatureSensorHandle)phid;
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
	case BP_SETRTDTYPE:
		if (!supportedRTDType(phid, (PhidgetTemperatureSensor_RTDType)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->RTDType = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "RTDType");
		break;
	case BP_SETRTDWIRESETUP:
		if (!supportedRTDWireSetup(phid, (Phidget_RTDWireSetup)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->RTDWireSetup = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "RTDWireSetup");
		break;
	case BP_SETCHANGETRIGGER:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minTemperatureChangeTrigger,
		  ch->maxTemperatureChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->temperatureChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "TemperatureChangeTrigger");
		break;
	case BP_SETTHERMOCOUPLETYPE:
		if (!supportedThermocoupleType(phid,
		  (PhidgetTemperatureSensor_ThermocoupleType)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->thermocoupleType = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "ThermocoupleType");
		break;
	case BP_TEMPERATURECHANGE:
		ch->temperature = getBridgePacketDouble(bp, 0);
		FIRECH(ch, TemperatureChange, ch->temperature);
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
	PhidgetTemperatureSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetTemperatureSensorHandle)phid;

	ret = EPHIDGET_OK;

	parentTemperatureSensor = (PhidgetTemperatureSensorDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 380;
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = -70;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 125;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = parentTemperatureSensor->maxTemperature[ch->phid.index];
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = parentTemperatureSensor->minTemperature[ch->phid.index];
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 150;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -55;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = parentTemperatureSensor->maxTemperature[ch->phid.index];
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = parentTemperatureSensor->minTemperature[ch->phid.index];
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 150;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -55;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = parentTemperatureSensor->maxTemperature[ch->phid.index];
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = parentTemperatureSensor->minTemperature[ch->phid.index];
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 150;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -55;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = parentTemperatureSensor->maxTemperature[ch->phid.index];
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = parentTemperatureSensor->minTemperature[ch->phid.index];
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 150;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -55;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = parentTemperatureSensor->maxTemperature[ch->phid.index];
		ch->maxTemperatureChangeTrigger = 1000;
		ch->minDataInterval = 32;
		ch->minTemperature = parentTemperatureSensor->minTemperature[ch->phid.index];
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 150;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 32;
		ch->minTemperature = -55;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = parentTemperatureSensor->temperature[ch->phid.index];
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 125;
		ch->maxTemperatureChangeTrigger = 165;
		ch->minDataInterval = 500;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 125;
		ch->maxTemperatureChangeTrigger = 165;
		ch->minDataInterval = 500;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 125;
		ch->maxTemperatureChangeTrigger = 165;
		ch->minDataInterval = 100;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 85;
		ch->maxTemperatureChangeTrigger = 125;
		ch->minDataInterval = 500;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 125;
		ch->maxTemperatureChangeTrigger = 165;
		ch->minDataInterval = 100;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 85;
		ch->maxTemperatureChangeTrigger = 125;
		ch->minDataInterval = 500;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 1370;
		ch->maxTemperatureChangeTrigger = 1640;
		ch->minDataInterval = 20;
		ch->minTemperature = -270;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 85;
		ch->maxTemperatureChangeTrigger = 125;
		ch->minDataInterval = 300;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 1370;
		ch->maxTemperatureChangeTrigger = 1640;
		ch->minDataInterval = 20;
		ch->minTemperature = -270;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		ch->thermocoupleType = THERMOCOUPLE_TYPE_K;
		break;
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 85;
		ch->maxTemperatureChangeTrigger = 125;
		ch->minDataInterval = 300;
		ch->minTemperature = -40;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 850;
		ch->maxTemperatureChangeTrigger = 1050;
		ch->minDataInterval = 250;
		ch->minTemperature = -200;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		ch->RTDType = RTD_TYPE_PT1000_3850;
		ch->RTDWireSetup = RTD_WIRE_SETUP_4WIRE;
		break;
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 100;
		ch->maxTemperatureChangeTrigger = 100;
		ch->minDataInterval = 20;
		ch->minTemperature = 0;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxTemperature = 85;
		ch->maxTemperatureChangeTrigger = 500;
		ch->minDataInterval = 20;
		ch->minTemperature = 0;
		ch->minTemperatureChangeTrigger = 0;
		ch->temperature = PUNK_DBL;
		ch->temperatureChangeTrigger = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetTemperatureSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetTemperatureSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
		break;
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
		break;
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
		break;
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
		  ch->thermocoupleType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [thermocoupleType] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETRTDTYPE, NULL, NULL, "%d", ch->RTDType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [RTDType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETRTDWIRESETUP, NULL, NULL, "%d", ch->RTDWireSetup);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [RTDWireSetup] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->temperatureChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [temperatureChangeTrigger] default: %d", phid, ret);
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
	PhidgetTemperatureSensorHandle ch;

	ch = (PhidgetTemperatureSensorHandle)phid;

	if(ch->temperature != PUNK_DBL)
		FIRECH(ch, TemperatureChange, ch->temperature);

}

static void CCONV
PhidgetTemperatureSensor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetTemperatureSensor));
}

API_PRETURN
PhidgetTemperatureSensor_create(PhidgetTemperatureSensorHandle *phidp) {

	CHANNELCREATE_BODY(TemperatureSensor, PHIDCHCLASS_TEMPERATURESENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_delete(PhidgetTemperatureSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetTemperatureSensor_setDataInterval(PhidgetTemperatureSensorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetTemperatureSensor_getDataInterval(PhidgetTemperatureSensorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMinDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMaxDataInterval(PhidgetTemperatureSensorHandle ch,
  uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_setRTDType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_RTDType RTDType) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETRTDTYPE, NULL, NULL, "%d", RTDType));
}

API_PRETURN
PhidgetTemperatureSensor_getRTDType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_RTDType *RTDType) {

	TESTPTRL(ch);
	TESTPTRL(RTDType);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*RTDType = ch->RTDType;
	if (ch->RTDType == (PhidgetTemperatureSensor_RTDType)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_setRTDWireSetup(PhidgetTemperatureSensorHandle ch,
  Phidget_RTDWireSetup RTDWireSetup) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETRTDWIRESETUP, NULL, NULL, "%d",
	  RTDWireSetup));
}

API_PRETURN
PhidgetTemperatureSensor_getRTDWireSetup(PhidgetTemperatureSensorHandle ch,
  Phidget_RTDWireSetup *RTDWireSetup) {

	TESTPTRL(ch);
	TESTPTRL(RTDWireSetup);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1048_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_000:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_200:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_300:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
	case PHIDCHUID_1051_TEMPERATURESENSOR_THERMOCOUPLE_400:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_THERMOCOUPLE_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*RTDWireSetup = ch->RTDWireSetup;
	if (ch->RTDWireSetup == (Phidget_RTDWireSetup)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getTemperature(PhidgetTemperatureSensorHandle ch, double *temperature) {

	TESTPTRL(ch);
	TESTPTRL(temperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*temperature = ch->temperature;
	if (ch->temperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMinTemperature(PhidgetTemperatureSensorHandle ch, double *minTemperature) {

	TESTPTRL(ch);
	TESTPTRL(minTemperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*minTemperature = ch->minTemperature;
	if (ch->minTemperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMaxTemperature(PhidgetTemperatureSensorHandle ch, double *maxTemperature) {

	TESTPTRL(ch);
	TESTPTRL(maxTemperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*maxTemperature = ch->maxTemperature;
	if (ch->maxTemperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_setTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double temperatureChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  temperatureChangeTrigger));
}

API_PRETURN
PhidgetTemperatureSensor_getTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *temperatureChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(temperatureChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*temperatureChangeTrigger = ch->temperatureChangeTrigger;
	if (ch->temperatureChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMinTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *minTemperatureChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minTemperatureChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*minTemperatureChangeTrigger = ch->minTemperatureChangeTrigger;
	if (ch->minTemperatureChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_getMaxTemperatureChangeTrigger(PhidgetTemperatureSensorHandle ch,
  double *maxTemperatureChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxTemperatureChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	*maxTemperatureChangeTrigger = ch->maxTemperatureChangeTrigger;
	if (ch->maxTemperatureChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_setThermocoupleType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_ThermocoupleType thermocoupleType) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETTHERMOCOUPLETYPE, NULL, NULL, "%d",
	  thermocoupleType));
}

API_PRETURN
PhidgetTemperatureSensor_getThermocoupleType(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_ThermocoupleType *thermocoupleType) {

	TESTPTRL(ch);
	TESTPTRL(thermocoupleType);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1045_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_1045_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1048_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_000:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_300:
	case PHIDCHUID_1051_TEMPERATURESENSOR_IC_400:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_DCC1000_TEMPERATURESENSOR_IC_200:
	case PHIDCHUID_DCC1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_HUM1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_SAF1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1000_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1100_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1101_TEMPERATURESENSOR_IC_100:
	case PHIDCHUID_TMP1200_TEMPERATURESENSOR_RTD_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IR_100:
	case PHIDCHUID_TMP1300_TEMPERATURESENSOR_IC_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*thermocoupleType = ch->thermocoupleType;
	if (ch->thermocoupleType == (PhidgetTemperatureSensor_ThermocoupleType)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetTemperatureSensor_setOnTemperatureChangeHandler(PhidgetTemperatureSensorHandle ch,
  PhidgetTemperatureSensor_OnTemperatureChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_TEMPERATURESENSOR);

	ch->TemperatureChange = fptr;
	ch->TemperatureChangeCtx = ctx;

	return (EPHIDGET_OK);
}
