/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetLightSensor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetLightSensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetLightSensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetLightSensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetLightSensor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetLightSensor {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double illuminance;
	double minIlluminance;
	double maxIlluminance;
	double illuminanceChangeTrigger;
	double minIlluminanceChangeTrigger;
	double maxIlluminanceChangeTrigger;
	PhidgetLightSensor_OnIlluminanceChangeCallback IlluminanceChange;
	void *IlluminanceChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetLightSensorHandle ch;
	int version;

	ch = (PhidgetLightSensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->illuminance = getBridgePacketDoubleByName(bp, "illuminance");
	ch->minIlluminance = getBridgePacketDoubleByName(bp, "minIlluminance");
	ch->maxIlluminance = getBridgePacketDoubleByName(bp, "maxIlluminance");
	ch->illuminanceChangeTrigger = getBridgePacketDoubleByName(bp, "illuminanceChangeTrigger");
	ch->minIlluminanceChangeTrigger = getBridgePacketDoubleByName(bp, "minIlluminanceChangeTrigger");
	ch->maxIlluminanceChangeTrigger = getBridgePacketDoubleByName(bp, "maxIlluminanceChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetLightSensorHandle ch;

	ch = (PhidgetLightSensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",illuminance=%g"
	  ",minIlluminance=%g"
	  ",maxIlluminance=%g"
	  ",illuminanceChangeTrigger=%g"
	  ",minIlluminanceChangeTrigger=%g"
	  ",maxIlluminanceChangeTrigger=%g"
	  ,0 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->illuminance
	  ,ch->minIlluminance
	  ,ch->maxIlluminance
	  ,ch->illuminanceChangeTrigger
	  ,ch->minIlluminanceChangeTrigger
	  ,ch->maxIlluminanceChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetLightSensorHandle)phid;
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
	case BP_SETCHANGETRIGGER:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minIlluminanceChangeTrigger,
		  ch->maxIlluminanceChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->illuminanceChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "IlluminanceChangeTrigger");
		break;
	case BP_ILLUMINANCECHANGE:
		ch->illuminance = getBridgePacketDouble(bp, 0);
		FIRECH(ch, IlluminanceChange, ch->illuminance);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetLightSensorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxIlluminance = 131072;
		ch->maxIlluminanceChangeTrigger = 131072;
		ch->minDataInterval = 125;
		ch->minIlluminance = 0;
		ch->minIlluminanceChangeTrigger = 0;
		ch->illuminance = PUNK_DBL;
		ch->illuminanceChangeTrigger = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetLightSensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetLightSensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_LUX1000_LIGHTSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->illuminanceChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [illuminanceChangeTrigger] default: %d", phid, ret);
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
	PhidgetLightSensorHandle ch;

	ch = (PhidgetLightSensorHandle)phid;

	if(ch->illuminance != PUNK_DBL)
		FIRECH(ch, IlluminanceChange, ch->illuminance);

}

static void CCONV
PhidgetLightSensor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetLightSensor));
}

API_PRETURN
PhidgetLightSensor_create(PhidgetLightSensorHandle *phidp) {

	CHANNELCREATE_BODY(LightSensor, PHIDCHCLASS_LIGHTSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_delete(PhidgetLightSensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetLightSensor_setDataInterval(PhidgetLightSensorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetLightSensor_getDataInterval(PhidgetLightSensorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinDataInterval(PhidgetLightSensorHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxDataInterval(PhidgetLightSensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getIlluminance(PhidgetLightSensorHandle ch, double *illuminance) {

	TESTPTRL(ch);
	TESTPTRL(illuminance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*illuminance = ch->illuminance;
	if (ch->illuminance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinIlluminance(PhidgetLightSensorHandle ch, double *minIlluminance) {

	TESTPTRL(ch);
	TESTPTRL(minIlluminance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*minIlluminance = ch->minIlluminance;
	if (ch->minIlluminance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxIlluminance(PhidgetLightSensorHandle ch, double *maxIlluminance) {

	TESTPTRL(ch);
	TESTPTRL(maxIlluminance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*maxIlluminance = ch->maxIlluminance;
	if (ch->maxIlluminance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_setIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double illuminanceChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  illuminanceChangeTrigger));
}

API_PRETURN
PhidgetLightSensor_getIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *illuminanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(illuminanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*illuminanceChangeTrigger = ch->illuminanceChangeTrigger;
	if (ch->illuminanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMinIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *minIlluminanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minIlluminanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*minIlluminanceChangeTrigger = ch->minIlluminanceChangeTrigger;
	if (ch->minIlluminanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_getMaxIlluminanceChangeTrigger(PhidgetLightSensorHandle ch,
  double *maxIlluminanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxIlluminanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);
	TESTATTACHEDL(ch);

	*maxIlluminanceChangeTrigger = ch->maxIlluminanceChangeTrigger;
	if (ch->maxIlluminanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetLightSensor_setOnIlluminanceChangeHandler(PhidgetLightSensorHandle ch,
  PhidgetLightSensor_OnIlluminanceChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_LIGHTSENSOR);

	ch->IlluminanceChange = fptr;
	ch->IlluminanceChangeCtx = ctx;

	return (EPHIDGET_OK);
}
