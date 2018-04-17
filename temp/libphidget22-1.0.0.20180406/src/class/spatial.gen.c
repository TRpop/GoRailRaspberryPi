/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/spatialdevice.h"
static void CCONV PhidgetSpatial_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetSpatial_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetSpatial_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetSpatial_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetSpatial_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetSpatial_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetSpatial_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetSpatial {
	struct _PhidgetChannel phid;
	uint32_t dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	PhidgetSpatial_OnSpatialDataCallback SpatialData;
	void *SpatialDataCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;
	int version;

	ch = (PhidgetSpatialHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		logerr("%"PRIphid": bad version %d != 1", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dataInterval = getBridgePacketUInt32ByName(bp, "dataInterval");
	ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetSpatialHandle ch;

	ch = (PhidgetSpatialHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ,1 /* class version */
	  ,ch->dataInterval
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetSpatialHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetSpatialHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETCORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_RESETCORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SAVECORRECTIONPARAMETERS:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_ZERO:
		res = DEVBRIDGEINPUT(phid, bp);
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
	case BP_SPATIALDATA:
		FIRECH(ch, SpatialData, getBridgePacketDoubleArray(bp, 0), getBridgePacketDoubleArray(bp, 1),
		  getBridgePacketDoubleArray(bp, 2), getBridgePacketDouble(bp, 3));
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetSpatialHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetSpatialHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		break;
	case PHIDCHUID_1044_SPATIAL_400:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		break;
	case PHIDCHUID_1056_SPATIAL_000:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		break;
	case PHIDCHUID_1056_SPATIAL_200:
		ch->dataInterval = 256;
		ch->maxDataInterval = 1000;
		ch->minDataInterval = 4;
		break;
	case PHIDCHUID_MOT1101_SPATIAL_100:
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
	PhidgetSpatialHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetSpatialHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1042_SPATIAL_300:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1044_SPATIAL_400:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1056_SPATIAL_000:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1056_SPATIAL_200:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, "%u", ch->dataInterval);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [dataInterval] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_MOT1101_SPATIAL_100:
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
PhidgetSpatial_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetSpatial));
}

API_PRETURN
PhidgetSpatial_create(PhidgetSpatialHandle *phidp) {

	CHANNELCREATE_BODY(Spatial, PHIDCHCLASS_SPATIAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_delete(PhidgetSpatialHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetSpatial_setMagnetometerCorrectionParameters(PhidgetSpatialHandle ch, double magneticField,
  double offset0, double offset1, double offset2, double gain0, double gain1, double gain2, double T0, double T1, double T2, double T3, double T4, double T5) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCORRECTIONPARAMETERS, NULL, NULL,
	  "%g%g%g%g%g%g%g%g%g%g%g%g%g", magneticField, offset0, offset1, offset2, gain0, gain1, gain2, T0, T1, T2, T3, T4, T5);
}

API_PRETURN
PhidgetSpatial_resetMagnetometerCorrectionParameters(PhidgetSpatialHandle ch) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_RESETCORRECTIONPARAMETERS, NULL, NULL,
	  NULL);
}

API_PRETURN
PhidgetSpatial_saveMagnetometerCorrectionParameters(PhidgetSpatialHandle ch) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SAVECORRECTIONPARAMETERS, NULL, NULL, NULL);
}

API_PRETURN
PhidgetSpatial_zeroGyro(PhidgetSpatialHandle ch) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_ZERO, NULL, NULL, NULL);
}

API_PRETURN
PhidgetSpatial_setDataInterval(PhidgetSpatialHandle ch, uint32_t dataInterval) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetSpatial_getDataInterval(PhidgetSpatialHandle ch, uint32_t *dataInterval) {

	TESTPTRL(ch);
	TESTPTRL(dataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	*dataInterval = ch->dataInterval;
	if (ch->dataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMinDataInterval(PhidgetSpatialHandle ch, uint32_t *minDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(minDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_getMaxDataInterval(PhidgetSpatialHandle ch, uint32_t *maxDataInterval) {

	TESTPTRL(ch);
	TESTPTRL(maxDataInterval);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);
	TESTATTACHEDL(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetSpatial_setOnSpatialDataHandler(PhidgetSpatialHandle ch, PhidgetSpatial_OnSpatialDataCallback fptr,
  void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_SPATIAL);

	ch->SpatialData = fptr;
	ch->SpatialDataCtx = ctx;

	return (EPHIDGET_OK);
}
