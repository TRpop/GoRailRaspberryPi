/* Generated: Wed Jun 22 2016 14:15:21 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/dataadapter.gen.h"
#include "class/dataadapter.gen.c"

static void CCONV
PhidgetDataAdapter_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDataAdapter_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDataAdapter_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDataAdapterHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetDataAdapterHandle)phid;

	switch (bp->vpkt) {
	case BP_DATAIN:
		FIRECH(ch, Packet, getBridgePacketUInt8Array(bp, 0), getBridgePacketArrayCnt(bp, 0), getBridgePacketUInt32(bp, 1));
		res = EPHIDGET_OK;
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetDataAdapter_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static PhidgetReturnCode
_sendData_async(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length, Phidget_AsyncCallback fptr, void *ctx) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_DATAADAPTER);
	TESTATTACHED(ch);

	if (length > ch->maxPacketLength)
		return (EPHIDGET_INVALIDARG);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DATAOUT, fptr, ctx, "%*R", length, data));
}

API_PRETURN
PhidgetDataAdapter_sendPacket(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length) {

	return (_sendData_async(ch, data, length, NULL, NULL));
}

API_VRETURN
PhidgetDataAdapter_sendPacket_async(PhidgetDataAdapterHandle ch, const uint8_t *data, size_t length,
  Phidget_AsyncCallback fptr, void *ctx) {

	PhidgetReturnCode res;

	res = _sendData_async(ch, data, length, fptr, ctx);
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}