/* Generated: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

#include "device/interfacekitdevice.h"
#include "device/encoderdevice.h"
#include "device/motorcontroldevice.h"
#include "device/stepperdevice.h"
#include "device/textlcddevice.h"
static void CCONV PhidgetDigitalInput_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static PhidgetReturnCode CCONV PhidgetDigitalInput_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDigitalInput_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetDigitalInput_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetDigitalInput_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetDigitalInput_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetDigitalInput_fireInitialEvents(PhidgetChannelHandle phid);

struct _PhidgetDigitalInput {
	struct _PhidgetChannel phid;
	Phidget_InputMode inputMode;
	Phidget_PowerSupply powerSupply;
	int state;
	PhidgetDigitalInput_OnStateChangeCallback StateChange;
	void *StateChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDigitalInputHandle ch;
	int version;

	ch = (PhidgetDigitalInputHandle)phid;

	version = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (version != 0) {
		logerr("%"PRIphid": bad version %d != 0", phid, version);
		return (EPHIDGET_BADVERSION);
	}

	ch->inputMode = getBridgePacketInt32ByName(bp, "inputMode");
	ch->powerSupply = getBridgePacketInt32ByName(bp, "powerSupply");
	ch->state = getBridgePacketInt32ByName(bp, "state");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetDigitalInputHandle ch;

	ch = (PhidgetDigitalInputHandle)phid;

	return (createBridgePacket(bp, 0, "_class_version_=%u"
	  ",inputMode=%d"
	  ",powerSupply=%d"
	  ",state=%d"
	  ,0 /* class version */
	  ,ch->inputMode
	  ,ch->powerSupply
	  ,ch->state
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDigitalInputHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDigitalInputHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
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
	case BP_STATECHANGE:
		ch->state = getBridgePacketInt32(bp, 0);
		FIRECH(ch, StateChange, ch->state);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetInterfaceKitDeviceHandle parentInterfaceKit;
	PhidgetMotorControlDeviceHandle parentMotorControl;
	PhidgetEncoderDeviceHandle parentEncoder;
	PhidgetStepperDeviceHandle parentStepper;
	PhidgetTextLCDDeviceHandle parentTextLCD;
	PhidgetDigitalInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);
	ch = (PhidgetDigitalInputHandle)phid;

	ret = EPHIDGET_OK;

	parentInterfaceKit = (PhidgetInterfaceKitDeviceHandle)phid->parent;
	parentMotorControl = (PhidgetMotorControlDeviceHandle)phid->parent;
	parentEncoder = (PhidgetEncoderDeviceHandle)phid->parent;
	parentStepper = (PhidgetStepperDeviceHandle)phid->parent;
	parentTextLCD = (PhidgetTextLCDDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1011_DIGITALINPUT_000:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1012_DIGITALINPUT_000:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1012_DIGITALINPUT_601:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1012_DIGITALINPUT_602:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1013_DIGITALINPUT_000:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1018_DIGITALINPUT_821:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1018_DIGITALINPUT_900:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1047_DIGITALINPUT_100:
		ch->state = parentEncoder->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1052_DIGITALINPUT_000:
		ch->state = parentEncoder->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1052_DIGITALINPUT_101:
		ch->state = parentEncoder->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1052_DIGITALINPUT_110:
		ch->state = parentEncoder->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1060_DIGITALINPUT_100:
		ch->state = parentMotorControl->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1063_DIGITALINPUT_100:
		ch->state = parentStepper->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1065_DIGITALINPUT_100:
		ch->state = parentMotorControl->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1202_DIGITALINPUT_000:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1202_DIGITALINPUT_120:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1202_DIGITALINPUT_300:
		ch->state = parentInterfaceKit->inputState[ch->phid.index];
		break;
	case PHIDCHUID_1219_DIGITALINPUT_000:
		ch->state = parentTextLCD->inputState[ch->phid.index];
		break;
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		ch->powerSupply = POWER_SUPPLY_12V;
		ch->inputMode = INPUT_MODE_NPN;
		break;
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		ch->state = PUNK_BOOL;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetDigitalInputHandle ch;
	PhidgetReturnCode ret;

	TESTPTRL(phid);

	ch = (PhidgetDigitalInputHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1011_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1012_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1012_DIGITALINPUT_601:
		break;
	case PHIDCHUID_1012_DIGITALINPUT_602:
		break;
	case PHIDCHUID_1013_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1018_DIGITALINPUT_821:
		break;
	case PHIDCHUID_1018_DIGITALINPUT_900:
		break;
	case PHIDCHUID_1047_DIGITALINPUT_100:
		break;
	case PHIDCHUID_1052_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1052_DIGITALINPUT_101:
		break;
	case PHIDCHUID_1052_DIGITALINPUT_110:
		break;
	case PHIDCHUID_1060_DIGITALINPUT_100:
		break;
	case PHIDCHUID_1063_DIGITALINPUT_100:
		break;
	case PHIDCHUID_1065_DIGITALINPUT_100:
		break;
	case PHIDCHUID_1202_DIGITALINPUT_000:
		break;
	case PHIDCHUID_1202_DIGITALINPUT_120:
		break;
	case PHIDCHUID_1202_DIGITALINPUT_300:
		break;
	case PHIDCHUID_1219_DIGITALINPUT_000:
		break;
	case PHIDCHUID_HUB_DIGITALINPUT_100:
		break;
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
		break;
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
		break;
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
		break;
	case PHIDCHUID_DAQ1400_DIGITALINPUT_100:
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
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
		break;
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetDigitalInputHandle ch;

	ch = (PhidgetDigitalInputHandle)phid;

	if(ch->state != PUNK_BOOL)
		FIRECH(ch, StateChange, ch->state);

}

static void CCONV
PhidgetDigitalInput_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetDigitalInput));
}

API_PRETURN
PhidgetDigitalInput_create(PhidgetDigitalInputHandle *phidp) {

	CHANNELCREATE_BODY(DigitalInput, PHIDCHCLASS_DIGITALINPUT);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalInput_delete(PhidgetDigitalInputHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetDigitalInput_setInputMode(PhidgetDigitalInputHandle ch, Phidget_InputMode inputMode) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETINPUTMODE, NULL, NULL, "%d",
	  inputMode));
}

API_PRETURN
PhidgetDigitalInput_getInputMode(PhidgetDigitalInputHandle ch, Phidget_InputMode *inputMode) {

	TESTPTRL(ch);
	TESTPTRL(inputMode);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
	case PHIDCHUID_1011_DIGITALINPUT_000:
	case PHIDCHUID_1012_DIGITALINPUT_000:
	case PHIDCHUID_1012_DIGITALINPUT_601:
	case PHIDCHUID_1012_DIGITALINPUT_602:
	case PHIDCHUID_1013_DIGITALINPUT_000:
	case PHIDCHUID_1018_DIGITALINPUT_821:
	case PHIDCHUID_1018_DIGITALINPUT_900:
	case PHIDCHUID_1047_DIGITALINPUT_100:
	case PHIDCHUID_1052_DIGITALINPUT_000:
	case PHIDCHUID_1052_DIGITALINPUT_101:
	case PHIDCHUID_1052_DIGITALINPUT_110:
	case PHIDCHUID_1060_DIGITALINPUT_100:
	case PHIDCHUID_1063_DIGITALINPUT_100:
	case PHIDCHUID_1065_DIGITALINPUT_100:
	case PHIDCHUID_1202_DIGITALINPUT_000:
	case PHIDCHUID_1202_DIGITALINPUT_120:
	case PHIDCHUID_1202_DIGITALINPUT_300:
	case PHIDCHUID_1219_DIGITALINPUT_000:
	case PHIDCHUID_HUB_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
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
PhidgetDigitalInput_setPowerSupply(PhidgetDigitalInputHandle ch, Phidget_PowerSupply powerSupply) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);
	TESTATTACHEDL(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETPOWERSUPPLY, NULL, NULL, "%d",
	  powerSupply));
}

API_PRETURN
PhidgetDigitalInput_getPowerSupply(PhidgetDigitalInputHandle ch, Phidget_PowerSupply *powerSupply) {

	TESTPTRL(ch);
	TESTPTRL(powerSupply);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);
	TESTATTACHEDL(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_ifkit488_DIGITALINPUT_000:
	case PHIDCHUID_1011_DIGITALINPUT_000:
	case PHIDCHUID_1012_DIGITALINPUT_000:
	case PHIDCHUID_1012_DIGITALINPUT_601:
	case PHIDCHUID_1012_DIGITALINPUT_602:
	case PHIDCHUID_1013_DIGITALINPUT_000:
	case PHIDCHUID_1018_DIGITALINPUT_821:
	case PHIDCHUID_1018_DIGITALINPUT_900:
	case PHIDCHUID_1047_DIGITALINPUT_100:
	case PHIDCHUID_1052_DIGITALINPUT_000:
	case PHIDCHUID_1052_DIGITALINPUT_101:
	case PHIDCHUID_1052_DIGITALINPUT_110:
	case PHIDCHUID_1060_DIGITALINPUT_100:
	case PHIDCHUID_1063_DIGITALINPUT_100:
	case PHIDCHUID_1065_DIGITALINPUT_100:
	case PHIDCHUID_1202_DIGITALINPUT_000:
	case PHIDCHUID_1202_DIGITALINPUT_120:
	case PHIDCHUID_1202_DIGITALINPUT_300:
	case PHIDCHUID_1219_DIGITALINPUT_000:
	case PHIDCHUID_HUB_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1200_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1300_DIGITALINPUT_100:
	case PHIDCHUID_DAQ1301_DIGITALINPUT_100:
	case PHIDCHUID_HIN1101_DIGITALINPUT_100:
	case PHIDCHUID_HIN1100_DIGITALINPUT_100:
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
PhidgetDigitalInput_getState(PhidgetDigitalInputHandle ch, int *state) {

	TESTPTRL(ch);
	TESTPTRL(state);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);
	TESTATTACHEDL(ch);

	*state = ch->state;
	if (ch->state == (int)PUNK_BOOL)
		return (EPHIDGET_UNKNOWNVAL);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetDigitalInput_setOnStateChangeHandler(PhidgetDigitalInputHandle ch,
  PhidgetDigitalInput_OnStateChangeCallback fptr, void *ctx) {

	TESTPTRL(ch);
	TESTCHANNELCLASSL(ch, PHIDCHCLASS_DIGITALINPUT);

	ch->StateChange = fptr;
	ch->StateChangeCtx = ctx;

	return (EPHIDGET_OK);
}
