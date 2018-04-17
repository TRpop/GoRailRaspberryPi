/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/phsensordevice.h"
static void CCONV PhidgetPHSensor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetPHSensor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetPHSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetPHSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetPHSensor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetPHSensor {
	struct _PhidgetChannel phid;
	double correctionTemperature;
	double minCorrectionTemperature;
	double maxCorrectionTemperature;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double PH;
	double minPH;
	double maxPH;
	double PHChangeTrigger;
	double minPHChangeTrigger;
	double maxPHChangeTrigger;
	PhidgetPHSensor_OnPHChangeCallback PHChange;
	void *PHChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPHSensorHandle ch;
	int version;

	ch = (PhidgetPHSensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->correctionTemperature = getBridgePacketDoubleByName(bp, "correctionTemperature");
	ch->minCorrectionTemperature = getBridgePacketDoubleByName(bp, "minCorrectionTemperature");
	ch->maxCorrectionTemperature = getBridgePacketDoubleByName(bp, "maxCorrectionTemperature");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->PH = getBridgePacketDoubleByName(bp, "PH");
	ch->minPH = getBridgePacketDoubleByName(bp, "minPH");
	ch->maxPH = getBridgePacketDoubleByName(bp, "maxPH");
	ch->PHChangeTrigger = getBridgePacketDoubleByName(bp, "PHChangeTrigger");
	ch->minPHChangeTrigger = getBridgePacketDoubleByName(bp, "minPHChangeTrigger");
	ch->maxPHChangeTrigger = getBridgePacketDoubleByName(bp, "maxPHChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetPHSensorHandle ch;

	ch = (PhidgetPHSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",correctionTemperature=%g"
	  ",minCorrectionTemperature=%g"
	  ",maxCorrectionTemperature=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",PH=%g"
	  ",minPH=%g"
	  ",maxPH=%g"
	  ",PHChangeTrigger=%g"
	  ",minPHChangeTrigger=%g"
	  ",maxPHChangeTrigger=%g"
	  ,0 /* class version */
	  ,ch->correctionTemperature
	  ,ch->minCorrectionTemperature
	  ,ch->maxCorrectionTemperature
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->PH
	  ,ch->minPH
	  ,ch->maxPH
	  ,ch->PHChangeTrigger
	  ,ch->minPHChangeTrigger
	  ,ch->maxPHChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetPHSensorHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETCORRECTIONTEMPERATURE:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minCorrectionTemperature,
		  ch->maxCorrectionTemperature);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->correctionTemperature = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "CorrectionTemperature");
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
	case BP_SETCHANGETRIGGER:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minPHChangeTrigger, ch->maxPHChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->PHChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "PHChangeTrigger");
		break;
	case BP_PHCHANGE:
		ch->PH = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PHChange, ch->PH);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetPHSensorDeviceHandle parentPHSensor;
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetPHSensorHandle)phid;

	ret = EPHIDGET_OK;

	parentPHSensor = (PhidgetPHSensorDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1058_PHADAPTER_100:
		ch->dataInterval = 256;
		ch->maxDataInterval = 60000;
		ch->maxPH = 14;
		ch->maxPHChangeTrigger = 14;
		ch->minDataInterval = 80;
		ch->minPH = 0;
		ch->minPHChangeTrigger = 0;
		ch->PH = parentPHSensor->PH[ch->phid.index];
		ch->PHChangeTrigger = 0;
		ch->correctionTemperature = 25;
		ch->minCorrectionTemperature = 0;
		ch->maxCorrectionTemperature = 100;
		break;
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxPH = 14;
		ch->maxPHChangeTrigger = 14;
		ch->minDataInterval = 50;
		ch->minPH = 0;
		ch->minPHChangeTrigger = 0;
		ch->PH = PUNK_DBL;
		ch->PHChangeTrigger = 0;
		ch->correctionTemperature = 25;
		ch->minCorrectionTemperature = 0;
		ch->maxCorrectionTemperature = 100;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetPHSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetPHSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1058_PHADAPTER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g", ch->PHChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [PHChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
		  ch->correctionTemperature);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [correctionTemperature] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_ADP1000_PHSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g", ch->PHChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [PHChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
		  ch->correctionTemperature);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [correctionTemperature] default: %d", phid, ret);
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
	PhidgetPHSensorHandle ch;

	ch = (PhidgetPHSensorHandle)phid;

	if(ch->PH != PUNK_DBL)
		FIRECH(ch, PHChange, ch->PH);

}

static void CCONV
PhidgetPHSensor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetPHSensor));
}

API_PRETURN
PhidgetPHSensor_create(PhidgetPHSensorHandle *phidp) {

	CHANNELCREATE_BODY(PHSensor, PHIDCHCLASS_PHSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_delete(PhidgetPHSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetPHSensor_setCorrectionTemperature(PhidgetPHSensorHandle ch, double correctionTemperature) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCORRECTIONTEMPERATURE, NULL, NULL, "%g",
	  correctionTemperature));
}

API_PRETURN
PhidgetPHSensor_getCorrectionTemperature(PhidgetPHSensorHandle ch, double *correctionTemperature) {

	TESTPTRL(ch);
	TESTPTRL(correctionTemperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*correctionTemperature = ch->correctionTemperature;
	if (ch->correctionTemperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinCorrectionTemperature(PhidgetPHSensorHandle ch,
  double *minCorrectionTemperature) {

	TESTPTRL(ch);
	TESTPTRL(minCorrectionTemperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*minCorrectionTemperature = ch->minCorrectionTemperature;
	if (ch->minCorrectionTemperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxCorrectionTemperature(PhidgetPHSensorHandle ch,
  double *maxCorrectionTemperature) {

	TESTPTRL(ch);
	TESTPTRL(maxCorrectionTemperature);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*maxCorrectionTemperature = ch->maxCorrectionTemperature;
	if (ch->maxCorrectionTemperature == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setDataInterval(PhidgetPHSensorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetPHSensor_getDataInterval(PhidgetPHSensorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinDataInterval(PhidgetPHSensorHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxDataInterval(PhidgetPHSensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getPH(PhidgetPHSensorHandle ch, double *PH) {

	TESTPTRL(ch);
	TESTPTRL(PH);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*PH = ch->PH;
	if (ch->PH == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinPH(PhidgetPHSensorHandle ch, double *minPH) {

	TESTPTRL(ch);
	TESTPTRL(minPH);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*minPH = ch->minPH;
	if (ch->minPH == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxPH(PhidgetPHSensorHandle ch, double *maxPH) {

	TESTPTRL(ch);
	TESTPTRL(maxPH);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*maxPH = ch->maxPH;
	if (ch->maxPH == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setPHChangeTrigger(PhidgetPHSensorHandle ch, double PHChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  PHChangeTrigger));
}

API_PRETURN
PhidgetPHSensor_getPHChangeTrigger(PhidgetPHSensorHandle ch, double *PHChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(PHChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*PHChangeTrigger = ch->PHChangeTrigger;
	if (ch->PHChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMinPHChangeTrigger(PhidgetPHSensorHandle ch, double *minPHChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minPHChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*minPHChangeTrigger = ch->minPHChangeTrigger;
	if (ch->minPHChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_getMaxPHChangeTrigger(PhidgetPHSensorHandle ch, double *maxPHChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxPHChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);
	TESTATTACHEDL(ch);

	*maxPHChangeTrigger = ch->maxPHChangeTrigger;
	if (ch->maxPHChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPHSensor_setOnPHChangeHandler(PhidgetPHSensorHandle ch, PhidgetPHSensor_OnPHChangeCallback fptr,
  void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_PHSENSOR);

	ch->PHChange = fptr;
	ch->PHChangeCtx = ctx;

	return (EPHIDGET_OK);
}
