/* Generated: Wed Oct 05 2016 13:13:43 GMT-0600 (Mountain Daylight Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/dictionary.gen.h"
#include "class/dictionary.gen.c"
#include "manager.h"

static void CCONV
PhidgetDictionary_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {
}

static PhidgetReturnCode CCONV
PhidgetDictionary_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDictionary_setDefaults(PhidgetChannelHandle phid) {
	return (_setDefaults(phid));
}

static PhidgetReturnCode
PhidgetDictionary_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {

	return (_bridgeInput(phid, bp));
}

static void CCONV
PhidgetDictionary_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

API_PRETURN
PhidgetDictionary_get(PhidgetDictionaryHandle ch, const char *key, char *value, size_t len) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED(ch);

	return (bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DICTIONARYGET, NULL, NULL, value,
	  (uint32_t)len, "%s", key));
}

API_PRETURN
PhidgetDictionary_scan(PhidgetDictionaryHandle ch, const char *startKey, char *keys, size_t len) {

	TESTPTR(ch);
	TESTCHANNELCLASS(ch, PHIDCHCLASS_DICTIONARY);
	TESTATTACHED(ch);

	if (startKey == NULL)
		startKey = "";

	return (bridgeSendToDeviceWithReply((PhidgetChannelHandle)ch, BP_DICTIONARYSCAN, NULL, NULL, keys,
	  (uint32_t)len, "%s", startKey));
}

API_PRETURN
PhidgetDictionary_enableStatsDictionary() {

	return (addDictionary(1, "Phidget22 Stats"));
}

API_PRETURN
PhidgetDictionary_enableControlDictionary() {

	return (addDictionary(2, "Phidget22 Control"));
}

PhidgetReturnCode
addDictionary(int serialNumber, const char *label) {
	static const PhidgetUniqueDeviceDef *pdd = NULL;
	PhidgetDeviceHandle device;
	PhidgetReturnCode res;

	if (pdd == NULL) {
		for (pdd = Phidget_Unique_Device_Def; ((int)pdd->type) != END_OF_LIST; pdd++) {
			if (pdd->id == PHIDID_DICTIONARY)
				goto founddictionary;
		}

		pdd = NULL;
		logerr("failed to find dictionary pdd");
		return (EPHIDGET_UNEXPECTED);
	}

founddictionary:

	/*
	 * Ensure the dictionary doesn't already exist.
	 */
	PhidgetLockDevices();
	FOREACH_DEVICE(device) {
		if (device->deviceInfo.class != PHIDCLASS_DICTIONARY)
			continue;

		if (device->deviceInfo.serialNumber == serialNumber) {
			PhidgetUnlockDevices();
			return (EPHIDGET_DUPLICATE);
		}
	}

	res = createPhidgetVirtualDevice(pdd, 100, label, serialNumber, &device);
	if (res != EPHIDGET_OK) {
		logerr("failed to create dictionary device\n");
		return (res);
	}

	res = deviceAttach(device, 0);
	if (res != EPHIDGET_OK)
		logerr("failed to attach dictionary device\n");

	PhidgetUnlockDevices();
	PhidgetRelease(&device);
	return (res);
}

API_PRETURN
PhidgetDictionary_addDictionary(int serialNumber, const char *label) {

	if (serialNumber < 1000) {
		logerr("serial number (%d) must be > 1000", serialNumber);
		return (EPHIDGET_INVALIDARG);
	}

	return (addDictionary(serialNumber, label));
}


API_PRETURN
PhidgetDictionary_removeDictionary(int serialNumber) {
	PhidgetDeviceHandle device;

	if (serialNumber < 1000) {
		logerr("serial number (%d) is not in the valid range (> 1000)", serialNumber);
		return (EPHIDGET_INVALIDARG);
	}

	PhidgetLockDevices();
	FOREACH_DEVICE(device) {
		if (device->deviceInfo.serialNumber == serialNumber) {
			chlog("%"PRIphid"", device);
			deviceDetach(device, 0);
			PhidgetUnlockDevices();
			return (EPHIDGET_OK);
		}
	}
	PhidgetUnlockDevices();

	return (EPHIDGET_NOENT);
}

#include "class/dictionary.gen.h"

API_PRETURN
PhidgetDictionary_loadDictionary(int serialNumber, const char *file) {
	PhidgetDictionaryHandle dict;
	PhidgetReturnCode res;
	char buf[2048];
	char val[2048];
	char key[255];
	char *sep;
	FILE *fp;

	logverbose("(%d) [%s]", serialNumber, file);

	fp = fopen(file, "r");
	if (fp == NULL) {
		logerr("failed to open '%s'", file);
		return (EPHIDGET_IO);
	}

	res = PhidgetDictionary_create(&dict);
	if (res != EPHIDGET_OK)
		return (res);

	res = Phidget_openWaitForAttachment((PhidgetHandle)dict, 1000);
	if (res != EPHIDGET_OK) {
		logerr("failed to open dictionary %d", serialNumber);
		goto bad;
	}

	while (fgets(buf, sizeof(buf), fp)) {
		if (buf[0] == '#')
			continue;
		sep = mos_strchr(buf, '=');
		if (sep == NULL)
			continue;

		*sep = '\0';
		sep++;
		mos_strtrim(buf, key, sizeof(key));
		mos_strtrim(sep, val, sizeof(val));
		if (mos_strlen(key) == 0 || mos_strlen(val) == 0)
			continue;

		logdebug("Dictionary %d set(%s=%s)", serialNumber, key, val);
		res = PhidgetDictionary_set(dict, key, val);
		if (res != EPHIDGET_OK)
			logerr("failed to set %s=<val> in dictionary %d: %d", key, serialNumber, res);
	}

	res = EPHIDGET_OK;

bad:

	fclose(fp);
	Phidget_close((PhidgetHandle)dict);
	PhidgetDictionary_delete(&dict);
	return (res);

}

