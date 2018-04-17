/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetHumiditySensor_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetHumiditySensor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetHumiditySensor_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetHumiditySensor {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double humidity;
	double minHumidity;
	double maxHumidity;
	double humidityChangeTrigger;
	double minHumidityChangeTrigger;
	double maxHumidityChangeTrigger;
	PhidgetHumiditySensor_OnHumidityChangeCallback HumidityChange;
	void *HumidityChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetHumiditySensorHandle ch;
	int version;

	ch = (PhidgetHumiditySensorHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->humidity = getBridgePacketDoubleByName(bp, "humidity");
	ch->minHumidity = getBridgePacketDoubleByName(bp, "minHumidity");
	ch->maxHumidity = getBridgePacketDoubleByName(bp, "maxHumidity");
	ch->humidityChangeTrigger = getBridgePacketDoubleByName(bp, "humidityChangeTrigger");
	ch->minHumidityChangeTrigger = getBridgePacketDoubleByName(bp, "minHumidityChangeTrigger");
	ch->maxHumidityChangeTrigger = getBridgePacketDoubleByName(bp, "maxHumidityChangeTrigger");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetHumiditySensorHandle ch;

	ch = (PhidgetHumiditySensorHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",humidity=%g"
	  ",minHumidity=%g"
	  ",maxHumidity=%g"
	  ",humidityChangeTrigger=%g"
	  ",minHumidityChangeTrigger=%g"
	  ",maxHumidityChangeTrigger=%g"
	  ,0 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->humidity
	  ,ch->minHumidity
	  ,ch->maxHumidity
	  ,ch->humidityChangeTrigger
	  ,ch->minHumidityChangeTrigger
	  ,ch->maxHumidityChangeTrigger
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetHumiditySensorHandle)phid;
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
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minHumidityChangeTrigger,
		  ch->maxHumidityChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->humidityChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "HumidityChangeTrigger");
		break;
	case BP_HUMIDITYCHANGE:
		ch->humidity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, HumidityChange, ch->humidity);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetHumiditySensorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		ch->dataInterval = 500;
		ch->maxDataInterval = 60000;
		ch->maxHumidity = 100;
		ch->maxHumidityChangeTrigger = 100;
		ch->minDataInterval = 500;
		ch->minHumidity = 0;
		ch->minHumidityChangeTrigger = 0;
		ch->humidity = PUNK_DBL;
		ch->humidityChangeTrigger = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetHumiditySensorHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetHumiditySensorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_HUM1000_HUMIDITYSENSOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->humidityChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [humidityChangeTrigger] default: %d", phid, ret);
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
	PhidgetHumiditySensorHandle ch;

	ch = (PhidgetHumiditySensorHandle)phid;

	if(ch->humidity != PUNK_DBL)
		FIRECH(ch, HumidityChange, ch->humidity);

}

static void CCONV
PhidgetHumiditySensor_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetHumiditySensor));
}

API_PRETURN
PhidgetHumiditySensor_create(PhidgetHumiditySensorHandle *phidp) {

	CHANNELCREATE_BODY(HumiditySensor, PHIDCHCLASS_HUMIDITYSENSOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_delete(PhidgetHumiditySensorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetHumiditySensor_setDataInterval(PhidgetHumiditySensorHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetHumiditySensor_getDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxDataInterval(PhidgetHumiditySensorHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getHumidity(PhidgetHumiditySensorHandle ch, double *humidity) {

	TESTPTRL(ch);
	TESTPTRL(humidity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*humidity = ch->humidity;
	if (ch->humidity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinHumidity(PhidgetHumiditySensorHandle ch, double *minHumidity) {

	TESTPTRL(ch);
	TESTPTRL(minHumidity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*minHumidity = ch->minHumidity;
	if (ch->minHumidity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxHumidity(PhidgetHumiditySensorHandle ch, double *maxHumidity) {

	TESTPTRL(ch);
	TESTPTRL(maxHumidity);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*maxHumidity = ch->maxHumidity;
	if (ch->maxHumidity == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_setHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double humidityChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  humidityChangeTrigger));
}

API_PRETURN
PhidgetHumiditySensor_getHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *humidityChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(humidityChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*humidityChangeTrigger = ch->humidityChangeTrigger;
	if (ch->humidityChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMinHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *minHumidityChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minHumidityChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*minHumidityChangeTrigger = ch->minHumidityChangeTrigger;
	if (ch->minHumidityChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_getMaxHumidityChangeTrigger(PhidgetHumiditySensorHandle ch,
  double *maxHumidityChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxHumidityChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);
	TESTATTACHEDL(ch);

	*maxHumidityChangeTrigger = ch->maxHumidityChangeTrigger;
	if (ch->maxHumidityChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetHumiditySensor_setOnHumidityChangeHandler(PhidgetHumiditySensorHandle ch,
  PhidgetHumiditySensor_OnHumidityChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_HUMIDITYSENSOR);

	ch->HumidityChange = fptr;
	ch->HumidityChangeCtx = ctx;

	return (EPHIDGET_OK);
}
