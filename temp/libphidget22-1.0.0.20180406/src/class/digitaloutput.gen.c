/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/interfacekitdevice.h"
#include "device/rfiddevice.h"
#include "device/leddevice.h"
#include "device/textlcddevice.h"
static void CCONV PhidgetDigitalOutput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetDigitalOutput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDigitalOutput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDigitalOutput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDigitalOutput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDigitalOutput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDigitalOutput_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetDigitalOutput {
	struct _PhidgetChannel phid;
	double dutyCycle;
	double minDutyCycle;
	double maxDutyCycle;
	double LEDCurrentLimit;
	double minLEDCurrentLimit;
	double maxLEDCurrentLimit;
	PhidgetDigitalOutput_LEDForwardVoltage LEDForwardVoltage;
	int state;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDigitalOutputHandle ch;
	int version;

	ch = (PhidgetDigitalOutputHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->dutyCycle = getBridgePacketDoubleByName(bp, "dutyCycle");
	ch->minDutyCycle = getBridgePacketDoubleByName(bp, "minDutyCycle");
	ch->maxDutyCycle = getBridgePacketDoubleByName(bp, "maxDutyCycle");
	ch->LEDCurrentLimit = getBridgePacketDoubleByName(bp, "LEDCurrentLimit");
	ch->minLEDCurrentLimit = getBridgePacketDoubleByName(bp, "minLEDCurrentLimit");
	ch->maxLEDCurrentLimit = getBridgePacketDoubleByName(bp, "maxLEDCurrentLimit");
	ch->LEDForwardVoltage = getBridgePacketInt32ByName(bp, "LEDForwardVoltage");
	ch->state = getBridgePacketInt32ByName(bp, "state");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDigitalOutputHandle ch;

	ch = (PhidgetDigitalOutputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",dutyCycle=%g"
	  ",minDutyCycle=%g"
	  ",maxDutyCycle=%g"
	  ",LEDCurrentLimit=%g"
	  ",minLEDCurrentLimit=%g"
	  ",maxLEDCurrentLimit=%g"
	  ",LEDForwardVoltage=%d"
	  ",state=%d"
	  ,0 /* class version */
	  ,ch->dutyCycle
	  ,ch->minDutyCycle
	  ,ch->maxDutyCycle
	  ,ch->LEDCurrentLimit
	  ,ch->minLEDCurrentLimit
	  ,ch->maxLEDCurrentLimit
	  ,ch->LEDForwardVoltage
	  ,ch->state
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDigitalOutputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDigitalOutputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETDUTYCYCLE:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minDutyCycle, ch->maxDutyCycle);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->dutyCycle = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "DutyCycle");
		break;
	case BP_SETLEDCURRENTLIMIT:
		TESTRANGE(getBridgePacketDouble(bp, 0), ch->minLEDCurrentLimit, ch->maxLEDCurrentLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->LEDCurrentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "LEDCurrentLimit");
		break;
	case BP_SETLEDFORWARDVOLTAGE:
		if (!supportedLEDForwardVoltage(phid,
		  (PhidgetDigitalOutput_LEDForwardVoltage)getBridgePacketInt32(bp, 0)))
			return (EPHIDGET_INVALIDARG);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->LEDForwardVoltage = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "LEDForwardVoltage");
		break;
	case BP_SETSTATE:
		TESTBOOL(getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK) {
			logerr("%"PRIphid": DEVBRIDGEINPUT() failed: %d", phid, res);
			break;
		}
		ch->state = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp))
			FIRE_PROPERTYCHANGE(ch, "State");
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetDigitalOutputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetDigitalOutputHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		ch->state = 0;
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		ch->state = 0;
		break;
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		ch->state = 0;
		break;
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		ch->state = 0;
		break;
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		ch->state = 0;
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		ch->state = 0;
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		ch->state = 0;
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		ch->state = 0;
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		ch->state = 0;
		break;
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->LEDCurrentLimit = 0.03;
		ch->minLEDCurrentLimit = 0.03;
		ch->maxLEDCurrentLimit = 0.03;
		ch->LEDForwardVoltage = LED_FORWARD_VOLTAGE_3_2V;
		ch->state = 0;
		break;
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->LEDCurrentLimit = 0.02;
		ch->minLEDCurrentLimit = 0;
		ch->maxLEDCurrentLimit = 0.08;
		ch->LEDForwardVoltage = LED_FORWARD_VOLTAGE_2_75V;
		ch->state = 0;
		break;
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->LEDCurrentLimit = 0.02;
		ch->minLEDCurrentLimit = 0;
		ch->maxLEDCurrentLimit = 0.08;
		ch->LEDForwardVoltage = LED_FORWARD_VOLTAGE_2_75V;
		ch->state = 0;
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		ch->state = 0;
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		ch->state = 0;
		break;
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		ch->state = 0;
		break;
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->state = 0;
		break;
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->state = 0;
		break;
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->LEDCurrentLimit = 0.02;
		ch->minLEDCurrentLimit = 0;
		ch->maxLEDCurrentLimit = 0.04;
		ch->LEDForwardVoltage = LED_FORWARD_VOLTAGE_3_2V;
		ch->state = 0;
		break;
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->state = 0;
		break;
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->state = 0;
		break;
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		ch->dutyCycle = 0;
		ch->maxDutyCycle = 1;
		ch->minDutyCycle = 0;
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
		ch->state = 0;
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		ch->state = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetDigitalOutputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetDigitalOutputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
		break;
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
		break;
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
		break;
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
		break;
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
		break;
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
		break;
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
		break;
	case PHIDCHUID_1030_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_1031_DIGITALOUTPUT_100:
		ret = bridgeSendToDevice(phid, BP_SETLEDCURRENTLIMIT, NULL, NULL, "%g", ch->LEDCurrentLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [LEDCurrentLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETLEDFORWARDVOLTAGE, NULL, NULL, "%d",
		  ch->LEDForwardVoltage);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [LEDForwardVoltage] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1032_DIGITALOUTPUT_200:
		ret = bridgeSendToDevice(phid, BP_SETLEDCURRENTLIMIT, NULL, NULL, "%g", ch->LEDCurrentLimit);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [LEDCurrentLimit] default: %d", phid, ret);
			break;
		}
		ret = bridgeSendToDevice(phid, BP_SETLEDFORWARDVOLTAGE, NULL, NULL, "%d",
		  ch->LEDForwardVoltage);
		if (ret != EPHIDGET_OK) {
			logerr("%"PRIphid": failed to set [LEDForwardVoltage] default: %d", phid, ret);
			break;
		}
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
		break;
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
		break;
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
		break;
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_LED1000_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
		break;
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
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
PhidgetDigitalOutput_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDigitalOutput));
}

API_PRETURN
PhidgetDigitalOutput_create(PhidgetDigitalOutputHandle *phidp) {

	CHANNELCREATE_BODY(DigitalOutput, PHIDCHCLASS_DIGITALOUTPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_delete(PhidgetDigitalOutputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDigitalOutput_setDutyCycle(PhidgetDigitalOutputHandle ch, double dutyCycle) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, "%g",
	  dutyCycle));
}

API_VRETURN
PhidgetDigitalOutput_setDutyCycle_async(PhidgetDigitalOutputHandle ch, double dutyCycle,
  Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_DIGITALOUTPUT) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, fptr, ctx, "%g", dutyCycle);
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetDigitalOutput_getDutyCycle(PhidgetDigitalOutputHandle ch, double *dutyCycle) {

	TESTPTRL(ch);
	TESTPTRL(dutyCycle);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*dutyCycle = ch->dutyCycle;
	if (ch->dutyCycle == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_getMinDutyCycle(PhidgetDigitalOutputHandle ch, double *minDutyCycle) {

	TESTPTRL(ch);
	TESTPTRL(minDutyCycle);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minDutyCycle = ch->minDutyCycle;
	if (ch->minDutyCycle == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_getMaxDutyCycle(PhidgetDigitalOutputHandle ch, double *maxDutyCycle) {

	TESTPTRL(ch);
	TESTPTRL(maxDutyCycle);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxDutyCycle = ch->maxDutyCycle;
	if (ch->maxDutyCycle == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_setLEDCurrentLimit(PhidgetDigitalOutputHandle ch, double LEDCurrentLimit) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETLEDCURRENTLIMIT, NULL, NULL, "%g",
	  LEDCurrentLimit));
}

API_VRETURN
PhidgetDigitalOutput_setLEDCurrentLimit_async(PhidgetDigitalOutputHandle ch, double LEDCurrentLimit,
  Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_DIGITALOUTPUT) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETLEDCURRENTLIMIT, fptr, ctx, "%g",
	  LEDCurrentLimit);
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetDigitalOutput_getLEDCurrentLimit(PhidgetDigitalOutputHandle ch, double *LEDCurrentLimit) {

	TESTPTRL(ch);
	TESTPTRL(LEDCurrentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*LEDCurrentLimit = ch->LEDCurrentLimit;
	if (ch->LEDCurrentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_getMinLEDCurrentLimit(PhidgetDigitalOutputHandle ch, double *minLEDCurrentLimit) {

	TESTPTRL(ch);
	TESTPTRL(minLEDCurrentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*minLEDCurrentLimit = ch->minLEDCurrentLimit;
	if (ch->minLEDCurrentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_getMaxLEDCurrentLimit(PhidgetDigitalOutputHandle ch, double *maxLEDCurrentLimit) {

	TESTPTRL(ch);
	TESTPTRL(maxLEDCurrentLimit);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*maxLEDCurrentLimit = ch->maxLEDCurrentLimit;
	if (ch->maxLEDCurrentLimit == (double)PUNK_DBL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_setLEDForwardVoltage(PhidgetDigitalOutputHandle ch,
  PhidgetDigitalOutput_LEDForwardVoltage LEDForwardVoltage) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETLEDFORWARDVOLTAGE, NULL, NULL, "%d",
	  LEDForwardVoltage));
}

API_PRETURN
PhidgetDigitalOutput_getLEDForwardVoltage(PhidgetDigitalOutputHandle ch,
  PhidgetDigitalOutput_LEDForwardVoltage *LEDForwardVoltage) {

	TESTPTRL(ch);
	TESTPTRL(LEDForwardVoltage);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALOUTPUT_000:
	case PHIDCHUID_1011_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_000:
	case PHIDCHUID_1012_DIGITALOUTPUT_601:
	case PHIDCHUID_1012_DIGITALOUTPUT_602:
	case PHIDCHUID_1013_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_821:
	case PHIDCHUID_1014_DIGITALOUTPUT_000:
	case PHIDCHUID_1014_DIGITALOUTPUT_704:
	case PHIDCHUID_1017_DIGITALOUTPUT_000:
	case PHIDCHUID_1018_DIGITALOUTPUT_900:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_200:
	case PHIDCHUID_1023_DIGITALOUTPUT_5V_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_LED_201:
	case PHIDCHUID_1023_DIGITALOUTPUT_ONBOARD_LED_201:
	case PHIDCHUID_1024_DIGITALOUTPUT_5V_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_LED_100:
	case PHIDCHUID_1024_DIGITALOUTPUT_ONBOARD_LED_100:
	case PHIDCHUID_1202_DIGITALOUTPUT_000:
	case PHIDCHUID_1202_DIGITALOUTPUT_120:
	case PHIDCHUID_1202_DIGITALOUTPUT_300:
	case PHIDCHUID_1219_DIGITALOUTPUT_000:
	case PHIDCHUID_HUB_DIGITALOUTPUT_100:
	case PHIDCHUID_OUT1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1000_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1100_DIGITALOUTPUT_100:
	case PHIDCHUID_REL1101_DIGITALOUTPUT_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_IN4_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A1_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A2_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A3_100:
	case PHIDCHUID_USBSWITCH_DIGITALOUTPUT_A4_100:
		return (EPHIDGET_UNSUPPORTED);
	default:
		break;
	}

	*LEDForwardVoltage = ch->LEDForwardVoltage;
	if (ch->LEDForwardVoltage == (PhidgetDigitalOutput_LEDForwardVoltage)PUNK_ENUM)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalOutput_setState(PhidgetDigitalOutputHandle ch, int state) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTATE, NULL, NULL, "%d", state));
}

API_VRETURN
PhidgetDigitalOutput_setState_async(PhidgetDigitalOutputHandle ch, int state, Phidget_AsyncCallback fptr,
  void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_DIGITALOUTPUT) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSTATE, fptr, ctx, "%d", state);
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetDigitalOutput_getState(PhidgetDigitalOutputHandle ch, int *state) {

	TESTPTRL(ch);
	TESTPTRL(state);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALOUTPUT);
	TESTATTACHEDL(ch);

	*state = ch->state;
	if (ch->state == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}
