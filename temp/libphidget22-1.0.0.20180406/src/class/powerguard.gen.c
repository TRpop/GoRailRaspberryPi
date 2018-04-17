/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

static void CCONV PhidgetPowerGuard_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetPowerGuard_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetPowerGuard_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetPowerGuard_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetPowerGuard_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetPowerGuard {
	struct _PhidgetChannel phid;
	Phidget_FanMode fanMode;
	double overVoltage;
	double minOverVoltage;
	double maxOverVoltage;
	int powerEnabled;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPowerGuardHandle ch;
	int version;

	ch = (PhidgetPowerGuardHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->fanMode = getBridgePacketInt32ByName(bp, "fanMode");
	ch->overVoltage = getBridgePacketDoubleByName(bp, "overVoltage");
	ch->minOverVoltage = getBridgePacketDoubleByName(bp, "minOverVoltage");
	ch->maxOverVoltage = getBridgePacketDoubleByName(bp, "maxOverVoltage");
	ch->powerEnabled = getBridgePacketInt32ByName(bp, "powerEnabled");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetPowerGuardHandle ch;

	ch = (PhidgetPowerGuardHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",fanMode=%d"
	  ",overVoltage=%g"
	  ",minOverVoltage=%g"
	  ",maxOverVoltage=%g"
	  ",powerEnabled=%d"
	  ,0 /* class version */
	  ,ch->fanMode
	  ,ch->overVoltage
	  ,ch->minOverVoltage
	  ,ch->maxOverVoltage
	  ,ch->powerEnabled
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetPowerGuardHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETFANMODE:
		if (!supportedFanMode(phid, (Phidget_FanMode)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->fanMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "FanMode");
		break;
	case BP_SETOVERVOLTAGE:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minOverVoltage, ch->maxOverVoltage);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->overVoltage = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "OverVoltage");
		break;
	case BP_SETENABLED:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->powerEnabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "PowerEnabled");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetPowerGuardHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		ch->powerEnabled = 0;
		ch->overVoltage = PUNK_DBL;
		ch->maxOverVoltage = 33;
		ch->minOverVoltage = 8;
		ch->fanMode = FAN_MODE_AUTO;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetPowerGuardHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetPowerGuardHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_SAF1000_POWERGUARD_100:
		ret = bridgeSendToDevice(phid, BP_SETENABLED, NULL, NULL, "%d", ch->powerEnabled);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [powerEnabled] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETFANMODE, NULL, NULL, "%d", ch->fanMode);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [fanMode] default: %d", phid, ret);
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
PhidgetPowerGuard_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetPowerGuard));
}

API_PRETURN
PhidgetPowerGuard_create(PhidgetPowerGuardHandle *phidp) {

	CHANNELCREATE_BODY(PowerGuard, PHIDCHCLASS_POWERGUARD);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_delete(PhidgetPowerGuardHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetPowerGuard_setFanMode(PhidgetPowerGuardHandle ch, Phidget_FanMode fanMode) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFANMODE, NULL, NULL, "%d", fanMode));
}

API_PRETURN
PhidgetPowerGuard_getFanMode(PhidgetPowerGuardHandle ch, Phidget_FanMode *fanMode) {

	TESTPTRL(ch);
	TESTPTRL(fanMode);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	*fanMode = ch->fanMode;
	if (ch->fanMode == (Phidget_FanMode)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_setOverVoltage(PhidgetPowerGuardHandle ch, double overVoltage) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETOVERVOLTAGE, NULL, NULL, "%g",
	  overVoltage));
}

API_PRETURN
PhidgetPowerGuard_getOverVoltage(PhidgetPowerGuardHandle ch, double *overVoltage) {

	TESTPTRL(ch);
	TESTPTRL(overVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	*overVoltage = ch->overVoltage;
	if (ch->overVoltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_getMinOverVoltage(PhidgetPowerGuardHandle ch, double *minOverVoltage) {

	TESTPTRL(ch);
	TESTPTRL(minOverVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	*minOverVoltage = ch->minOverVoltage;
	if (ch->minOverVoltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_getMaxOverVoltage(PhidgetPowerGuardHandle ch, double *maxOverVoltage) {

	TESTPTRL(ch);
	TESTPTRL(maxOverVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	*maxOverVoltage = ch->maxOverVoltage;
	if (ch->maxOverVoltage == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetPowerGuard_setPowerEnabled(PhidgetPowerGuardHandle ch, int powerEnabled) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLED, NULL, NULL, "%d",
	  powerEnabled));
}

API_PRETURN
PhidgetPowerGuard_getPowerEnabled(PhidgetPowerGuardHandle ch, int *powerEnabled) {

	TESTPTRL(ch);
	TESTPTRL(powerEnabled);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_POWERGUARD);
	TESTATTACHEDL(ch);

	*powerEnabled = ch->powerEnabled;
	if (ch->powerEnabled == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}
