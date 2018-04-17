/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/frequencycounterdevice.h"
static void CCONV PhidgetFrequencyCounter_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetFrequencyCounter_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetFrequencyCounter_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetFrequencyCounter_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetFrequencyCounter_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetFrequencyCounter_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetFrequencyCounter_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetFrequencyCounter {
	struct _PhidgetChannel phid;
	double totalTicksSinceLastCount;
	uint32_t frequencyPrecision;
	uint64_t count;
	int enabled;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	PhidgetFrequencyCounter_FilterType filterType;
	double frequency;
	double maxFrequency;
	double frequencyCutoff;
	double minFrequencyCutoff;
	double maxFrequencyCutoff;
	Phidget_InputMode inputMode;
	Phidget_PowerSupply powerSupply;
	double timeElapsed;
	PhidgetFrequencyCounter_OnCountChangeCallback CountChange;
	void *CountChangeCtx;
	PhidgetFrequencyCounter_OnFrequencyChangeCallback FrequencyChange;
	void *FrequencyChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetFrequencyCounterHandle ch;
	int version;

	ch = (PhidgetFrequencyCounterHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 2) {
		logerr("%"PRIphid": bad version %d != 2", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->totalTicksSinceLastCount = getBridgePacketDoubleByName(bp, "totalTicksSinceLastCount");
	ch->frequencyPrecision = getBridgePacketUInt32ByName(bp, "frequencyPrecision");
	ch->count = getBridgePacketUInt64ByName(bp, "count");
	ch->enabled = getBridgePacketInt32ByName(bp, "enabled");
	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	ch->filterType = getBridgePacketInt32ByName(bp, "filterType");
	ch->frequency = getBridgePacketDoubleByName(bp, "frequency");
	ch->maxFrequency = getBridgePacketDoubleByName(bp, "maxFrequency");
	ch->frequencyCutoff = getBridgePacketDoubleByName(bp, "frequencyCutoff");
	ch->minFrequencyCutoff = getBridgePacketDoubleByName(bp, "minFrequencyCutoff");
	ch->maxFrequencyCutoff = getBridgePacketDoubleByName(bp, "maxFrequencyCutoff");
	ch->inputMode = getBridgePacketInt32ByName(bp, "inputMode");
	ch->powerSupply = getBridgePacketInt32ByName(bp, "powerSupply");
	ch->timeElapsed = getBridgePacketDoubleByName(bp, "timeElapsed");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetFrequencyCounterHandle ch;

	ch = (PhidgetFrequencyCounterHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",totalTicksSinceLastCount=%g"
	  ",frequencyPrecision=%u"
	  ",count=%ul"
	  ",enabled=%d"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",filterType=%d"
	  ",frequency=%g"
	  ",maxFrequency=%g"
	  ",frequencyCutoff=%g"
	  ",minFrequencyCutoff=%g"
	  ",maxFrequencyCutoff=%g"
	  ",inputMode=%d"
	  ",powerSupply=%d"
	  ",timeElapsed=%g"
	  ,2 /* class version */
	  ,ch->totalTicksSinceLastCount
	  ,ch->frequencyPrecision
	  ,ch->count
	  ,ch->enabled
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->filterType
	  ,ch->frequency
	  ,ch->maxFrequency
	  ,ch->frequencyCutoff
	  ,ch->minFrequencyCutoff
	  ,ch->maxFrequencyCutoff
	  ,ch->inputMode
	  ,ch->powerSupply
	  ,ch->timeElapsed
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetFrequencyCounterHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetFrequencyCounterHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETENABLED:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->enabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "Enabled");
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
	case BP_SETFILTERTYPE:
		if (!supportedFilterType(phid, (PhidgetFrequencyCounter_FilterType)getBridgePacketInt32(bp,
		  0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->filterType = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "FilterType");
		break;
	case BP_SETINPUTMODE:
		if (!supportedInputMode(phid, (Phidget_InputMode)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->inputMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "InputMode");
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
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetFrequencyCounterHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetFrequencyCounterHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		ch->dataInterval = 256;
		ch->minDataInterval = 32;
		ch->maxDataInterval = 60000;
		ch->maxFrequency = 1000000;
		ch->maxFrequencyCutoff = 10;
		ch->minFrequencyCutoff = 0.01;
		ch->count = 0;
		ch->timeElapsed = 0;
		ch->frequencyCutoff = 1;
		ch->frequency = 0;
		ch->filterType = FILTER_TYPE_LOGIC_LEVEL;
		ch->enabled = 1;
		break;
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->maxFrequency = 1000000;
		ch->maxFrequencyCutoff = 10;
		ch->minFrequencyCutoff = 0.01;
		ch->count = 0;
		ch->timeElapsed = 0;
		ch->frequencyCutoff = 1;
		ch->frequency = 0;
		ch->filterType = FILTER_TYPE_LOGIC_LEVEL;
		ch->powerSupply = POWER_SUPPLY_12V;
		ch->inputMode = INPUT_MODE_NPN;
		ch->enabled = 1;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->totalTicksSinceLastCount = 0;
	ch->frequencyPrecision = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetFrequencyCounterHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetFrequencyCounterHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETFILTERTYPE, NULL, NULL, "%d", ch->filterType);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [filterType] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->enabled);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [enabled] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_DAQ1400_FREQUENCYCOUNTER_100:
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
		ret = bridgeSendToDevice(phid, BP_SETINPUTMODE, NULL, NULL, "%d", ch->inputMode);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [inputMode] default: %d", phid, ret);
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
PhidgetFrequencyCounter_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetFrequencyCounter));
}

API_PRETURN
PhidgetFrequencyCounter_create(PhidgetFrequencyCounterHandle *phidp) {

	CHANNELCREATE_BODY(FrequencyCounter, PHIDCHCLASS_FREQUENCYCOUNTER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_delete(PhidgetFrequencyCounterHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetFrequencyCounter_getCount(PhidgetFrequencyCounterHandle ch, uint64_t *count) {

	TESTPTRL(ch);
	TESTPTRL(count);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*count = ch->count;
	if (ch->count == (uint64_t)PUNK_UINT64)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setEnabled(PhidgetFrequencyCounterHandle ch, int enabled) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLED, NULL, NULL, "%d", enabled));
}

API_PRETURN
PhidgetFrequencyCounter_getEnabled(PhidgetFrequencyCounterHandle ch, int *enabled) {

	TESTPTRL(ch);
	TESTPTRL(enabled);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*enabled = ch->enabled;
	if (ch->enabled == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setDataInterval(PhidgetFrequencyCounterHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetFrequencyCounter_getDataInterval(PhidgetFrequencyCounterHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getMinDataInterval(PhidgetFrequencyCounterHandle ch,
  uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getMaxDataInterval(PhidgetFrequencyCounterHandle ch,
  uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setFilterType(PhidgetFrequencyCounterHandle ch,
  PhidgetFrequencyCounter_FilterType filterType) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFILTERTYPE, NULL, NULL, "%d",
	  filterType));
}

API_PRETURN
PhidgetFrequencyCounter_getFilterType(PhidgetFrequencyCounterHandle ch,
  PhidgetFrequencyCounter_FilterType *filterType) {

	TESTPTRL(ch);
	TESTPTRL(filterType);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*filterType = ch->filterType;
	if (ch->filterType == (PhidgetFrequencyCounter_FilterType)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getFrequency(PhidgetFrequencyCounterHandle ch, double *frequency) {

	TESTPTRL(ch);
	TESTPTRL(frequency);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*frequency = ch->frequency;
	if (ch->frequency == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getMaxFrequency(PhidgetFrequencyCounterHandle ch, double *maxFrequency) {

	TESTPTRL(ch);
	TESTPTRL(maxFrequency);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*maxFrequency = ch->maxFrequency;
	if (ch->maxFrequency == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getFrequencyCutoff(PhidgetFrequencyCounterHandle ch, double *frequencyCutoff) {

	TESTPTRL(ch);
	TESTPTRL(frequencyCutoff);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*frequencyCutoff = ch->frequencyCutoff;
	if (ch->frequencyCutoff == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getMinFrequencyCutoff(PhidgetFrequencyCounterHandle ch,
  double *minFrequencyCutoff) {

	TESTPTRL(ch);
	TESTPTRL(minFrequencyCutoff);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*minFrequencyCutoff = ch->minFrequencyCutoff;
	if (ch->minFrequencyCutoff == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_getMaxFrequencyCutoff(PhidgetFrequencyCounterHandle ch,
  double *maxFrequencyCutoff) {

	TESTPTRL(ch);
	TESTPTRL(maxFrequencyCutoff);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*maxFrequencyCutoff = ch->maxFrequencyCutoff;
	if (ch->maxFrequencyCutoff == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setInputMode(PhidgetFrequencyCounterHandle ch, Phidget_InputMode inputMode) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETINPUTMODE, NULL, NULL, "%d",
	  inputMode));
}

API_PRETURN
PhidgetFrequencyCounter_getInputMode(PhidgetFrequencyCounterHandle ch, Phidget_InputMode *inputMode) {

	TESTPTRL(ch);
	TESTPTRL(inputMode);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*inputMode = ch->inputMode;
	if (ch->inputMode == (Phidget_InputMode)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setPowerSupply(PhidgetFrequencyCounterHandle ch,
  Phidget_PowerSupply powerSupply) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPOWERSUPPLY, NULL, NULL, "%d",
	  powerSupply));
}

API_PRETURN
PhidgetFrequencyCounter_getPowerSupply(PhidgetFrequencyCounterHandle ch,
  Phidget_PowerSupply *powerSupply) {

	TESTPTRL(ch);
	TESTPTRL(powerSupply);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_1054_FREQUENCYCOUNTER_000:
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
PhidgetFrequencyCounter_getTimeElapsed(PhidgetFrequencyCounterHandle ch, double *timeElapsed) {

	TESTPTRL(ch);
	TESTPTRL(timeElapsed);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);
	TESTATTACHEDL(ch);

	*timeElapsed = ch->timeElapsed;
	if (ch->timeElapsed == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setOnCountChangeHandler(PhidgetFrequencyCounterHandle ch,
  PhidgetFrequencyCounter_OnCountChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);

	ch->CountChange = fptr;
	ch->CountChangeCtx = ctx;

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetFrequencyCounter_setOnFrequencyChangeHandler(PhidgetFrequencyCounterHandle ch,
  PhidgetFrequencyCounter_OnFrequencyChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_FREQUENCYCOUNTER);

	ch->FrequencyChange = fptr;
	ch->FrequencyChangeCtx = ctx;

	return (EPHIDGET_OK);
}
