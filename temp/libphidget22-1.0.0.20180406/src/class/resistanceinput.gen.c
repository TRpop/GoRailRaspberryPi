/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetResistanceInput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetResistanceInput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetResistanceInput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetResistanceInput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetResistanceInput_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetResistanceInput {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double resistance;
	double minResistance;
	double maxResistance;
	double resistanceChangeTrigger;
	double minResistanceChangeTrigger;
	double maxResistanceChangeTrigger;
	Phidget_RTDWireSetup RTDWireSetup;
	PhidgetResistanceInput_OnResistanceChangeCallback ResistanceChange;
	void *ResistanceChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetResistanceInputHandle ch;
	int version;

	ch = (PhidgetResistanceInputHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->resistance = getBridgePacketDoubleByName(bp, "resistance");
	ch->minResistance = getBridgePacketDoubleByName(bp, "minResistance");
	ch->maxResistance = getBridgePacketDoubleByName(bp, "maxResistance");
	ch->resistanceChangeTrigger = getBridgePacketDoubleByName(bp, "resistanceChangeTrigger");
	ch->minResistanceChangeTrigger = getBridgePacketDoubleByName(bp, "minResistanceChangeTrigger");
	ch->maxResistanceChangeTrigger = getBridgePacketDoubleByName(bp, "maxResistanceChangeTrigger");
	ch->RTDWireSetup = getBridgePacketInt32ByName(bp, "RTDWireSetup");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetResistanceInputHandle ch;

	ch = (PhidgetResistanceInputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",resistance=%g"
	  ",minResistance=%g"
	  ",maxResistance=%g"
	  ",resistanceChangeTrigger=%g"
	  ",minResistanceChangeTrigger=%g"
	  ",maxResistanceChangeTrigger=%g"
	  ",RTDWireSetup=%d"
	  ,0 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->resistance
	  ,ch->minResistance
	  ,ch->maxResistance
	  ,ch->resistanceChangeTrigger
	  ,ch->minResistanceChangeTrigger
	  ,ch->maxResistanceChangeTrigger
	  ,ch->RTDWireSetup
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetResistanceInputHandle)phid;
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
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minResistanceChangeTrigger,
		  ch->maxResistanceChangeTrigger);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->resistanceChangeTrigger = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "ResistanceChangeTrigger");
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
	case BP_RESISTANCECHANGE:
		ch->resistance = getBridgePacketDouble(bp, 0);
		FIRECH(ch, ResistanceChange, ch->resistance);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetResistanceInputHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		ch->dataInterval = 250;
		ch->maxDataInterval = 60000;
		ch->maxResistance = 50000;
		ch->maxResistanceChangeTrigger = 19000;
		ch->minDataInterval = 250;
		ch->minResistance = 0;
		ch->minResistanceChangeTrigger = 0;
		ch->resistance = PUNK_DBL;
		ch->resistanceChangeTrigger = 0;
		ch->RTDWireSetup = RTD_WIRE_SETUP_4WIRE;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetResistanceInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetResistanceInputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_TMP1200_RESISTANCEINPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
		  ch->resistanceChangeTrigger);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [resistanceChangeTrigger] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETRTDWIRESETUP, NULL, NULL, "%d", ch->RTDWireSetup);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [RTDWireSetup] default: %d", phid, ret);
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
	PhidgetResistanceInputHandle ch;

	ch = (PhidgetResistanceInputHandle)phid;

	if(ch->resistance != PUNK_DBL)
		FIRECH(ch, ResistanceChange, ch->resistance);

}

static void CCONV
PhidgetResistanceInput_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetResistanceInput));
}

API_PRETURN
PhidgetResistanceInput_create(PhidgetResistanceInputHandle *phidp) {

	CHANNELCREATE_BODY(ResistanceInput, PHIDCHCLASS_RESISTANCEINPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_delete(PhidgetResistanceInputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetResistanceInput_setDataInterval(PhidgetResistanceInputHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetResistanceInput_getDataInterval(PhidgetResistanceInputHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinDataInterval(PhidgetResistanceInputHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxDataInterval(PhidgetResistanceInputHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getResistance(PhidgetResistanceInputHandle ch, double *resistance) {

	TESTPTRL(ch);
	TESTPTRL(resistance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*resistance = ch->resistance;
	if (ch->resistance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinResistance(PhidgetResistanceInputHandle ch, double *minResistance) {

	TESTPTRL(ch);
	TESTPTRL(minResistance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*minResistance = ch->minResistance;
	if (ch->minResistance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxResistance(PhidgetResistanceInputHandle ch, double *maxResistance) {

	TESTPTRL(ch);
	TESTPTRL(maxResistance);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*maxResistance = ch->maxResistance;
	if (ch->maxResistance == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double resistanceChangeTrigger) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCHANGETRIGGER, NULL, NULL, "%g",
	  resistanceChangeTrigger));
}

API_PRETURN
PhidgetResistanceInput_getResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *resistanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(resistanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*resistanceChangeTrigger = ch->resistanceChangeTrigger;
	if (ch->resistanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMinResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *minResistanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(minResistanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*minResistanceChangeTrigger = ch->minResistanceChangeTrigger;
	if (ch->minResistanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_getMaxResistanceChangeTrigger(PhidgetResistanceInputHandle ch,
  double *maxResistanceChangeTrigger) {

	TESTPTRL(ch);
	TESTPTRL(maxResistanceChangeTrigger);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*maxResistanceChangeTrigger = ch->maxResistanceChangeTrigger;
	if (ch->maxResistanceChangeTrigger == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setRTDWireSetup(PhidgetResistanceInputHandle ch,
  Phidget_RTDWireSetup RTDWireSetup) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETRTDWIRESETUP, NULL, NULL, "%d",
	  RTDWireSetup));
}

API_PRETURN
PhidgetResistanceInput_getRTDWireSetup(PhidgetResistanceInputHandle ch,
  Phidget_RTDWireSetup *RTDWireSetup) {

	TESTPTRL(ch);
	TESTPTRL(RTDWireSetup);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);
	TESTATTACHEDL(ch);

	*RTDWireSetup = ch->RTDWireSetup;
	if (ch->RTDWireSetup == (Phidget_RTDWireSetup)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetResistanceInput_setOnResistanceChangeHandler(PhidgetResistanceInputHandle ch,
  PhidgetResistanceInput_OnResistanceChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_RESISTANCEINPUT);

	ch->ResistanceChange = fptr;
	ch->ResistanceChangeCtx = ctx;

	return (EPHIDGET_OK);
}
