/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/hub.gen.h"
#include "class/hub.gen.c"

static void
PhidgetHub_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static PhidgetReturnCode CCONV
PhidgetHub_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHub_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetHub_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetHub_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetHub_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	switch (bp->vpkt) {
	case BP_SETFIRMWAREUPGRADEFLAG:
		TESTRANGE(getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		TESTRANGE(getBridgePacketUInt32(bp, 1), 0, 0xFFFF);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETPORTMODE:
		TESTRANGE(getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		if (!supportedPortMode(phid, (PhidgetHub_PortMode)getBridgePacketInt32(bp, 1)))
			return (EPHIDGET_INVALIDARG);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETPORTPOWER:
		TESTRANGE(getBridgePacketInt32(bp, 0), 0, phid->parent->dev_hub.numVintPorts);
		TESTBOOL(getBridgePacketInt32(bp, 1));
		res = _bridgeInput(phid, bp);
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetHub_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}
