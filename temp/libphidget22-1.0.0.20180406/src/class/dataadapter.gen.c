/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetDataAdapter_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetDataAdapter {
	struct _PhidgetChannel phid;
	uint32_t maxPacketLength;
	PhidgetDataAdapter_OnPacketCallback Packet;
	void *PacketCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	int version;

	ch = (PhidgetDataAdapterHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 1) {
		logerr("%"PRIphid": bad version %d != 1", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->maxPacketLength = getBridgePacketUInt32ByName(bp, "maxPacketLength");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDataAdapterHandle ch;

	ch = (PhidgetDataAdapterHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",maxPacketLength=%u"
	  ,1 /* class version */
	  ,ch->maxPacketLength
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_DATAOUT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetDataAdapterHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
		ch->maxPacketLength = 48;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ADP1001_DATAADAPTER_100:
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
PhidgetDataAdapter_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDataAdapter));
}

API_PRETURN
PhidgetDataAdapter_create(PhidgetDataAdapterHandle *phidp) {

	CHANNELCREATE_BODY(DataAdapter, PHIDCHCLASS_DATAADAPTER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_delete(PhidgetDataAdapterHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDataAdapter_getMaxPacketLength(PhidgetDataAdapterHandle ch, uint32_t *maxPacketLength) {

	TESTPTRL(ch);
	TESTPTRL(maxPacketLength);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHEDL(ch);

	*maxPacketLength = ch->maxPacketLength;
	if (ch->maxPacketLength == (uint32_t)PUNK_UINT32)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDataAdapter_setOnPacketHandler(PhidgetDataAdapterHandle ch,
  PhidgetDataAdapter_OnPacketCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DATAADAPTER);

	ch->Packet = fptr;
	ch->PacketCtx = ctx;

	return (EPHIDGET_OK);
}
