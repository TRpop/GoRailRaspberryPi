#ifndef EXTERNALPROTO
/*
* This file is part of libphidget22
*
* Copyright 2015 Phidgets Inc <patrick@phidgets.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see
* <http://www.gnu.org/licenses/>
*/
#endif

#ifndef _CPHIDGET_MACROS
#define _CPHIDGET_MACROS

#define TESTPTR(arg)		if (arg == NULL) return (EPHIDGET_INVALIDARG);

#define TESTPTRL(arg)		do {										\
	if (arg == NULL) {													\
		logerr("NULL pointer detected");								\
		return (EPHIDGET_INVALIDARG);									\
	}																	\
} while (0)

#define TESTINDEX(vname1)	if (Index >= phid->vname1 || Index < 0) return (EPHIDGET_INVALIDARG);
#define TESTMASGN(vname1, check)	if (phid->vname1 == check) {*pVal = check; return (EPHIDGET_UNKNOWNVAL);}

#define TESTRANGE(newVal, min, max)	do {								\
	if (newVal < (min) || newVal > (max)) return (EPHIDGET_INVALIDARG);	\
} while (0)

#define TESTBOOL(newVal)	do {								\
	if (newVal != (PFALSE) && newVal != (PTRUE)) return (EPHIDGET_INVALIDARG);	\
} while (0)

#define TESTLESSTHEN(newVal, max)	if (newVal > (max)) return (EPHIDGET_INVALIDARG);

#define TESTCHANNELCLASS(ch, def)	\
	if (((PhidgetChannelHandle)(ch))->class != def) return (EPHIDGET_WRONGDEVICE);

#define TESTCHANNELCLASSL(ch, def)	do {											\
	if (((PhidgetChannelHandle)(ch))->class != def) {								\
		logerr("channel class %d != %d", ((PhidgetChannelHandle)(ch))->class, def);	\
		return (EPHIDGET_WRONGDEVICE);												\
	}																				\
} while (0)

#define TESTATTACHED(phid) do {											\
	if (!ISATTACHED(phid))												\
		return (EPHIDGET_NOTATTACHED);									\
} while (0)

#define TESTATTACHEDL(phid) do {										\
	if (!ISATTACHED(phid)) {											\
		logerr("%"PRIphid" is not attached", phid);								\
		return (EPHIDGET_NOTATTACHED);									\
	}																	\
} while (0)

#define TESTATTACHEDORDETACHING(phid) do {								\
	if (!ISATTACHEDORDETACHING(phid))									\
		return (EPHIDGET_NOTATTACHED);									\
} while (0)

#define TESTOPEN(phid) do {												\
	if (!ISOPEN(phid))													\
		return (EPHIDGET_INVALIDARG);									\
} while (0)

#define TESTCHANNEL(ch) do {											\
	if (!ISCHANNEL(ch))													\
		return (EPHIDGET_INVALIDARG);									\
} while (0)

#define FIRECH(ch, ename, ...) do {																	\
	if ((ch)->ename)																				\
		(ch)->ename((ch), (ch)->ename##Ctx, __VA_ARGS__);											\
} while (0)

#define FIRECH0(ch, ename) do {																		\
	if ((ch)->ename)																				\
		(ch)->ename((ch), (ch)->ename##Ctx);														\
} while (0)

#define FIRE_PROPERTYCHANGE(ch, prop) do {																\
	PhidgetChannelHandle _ch_;																		\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_ && _ch_->PropertyChange)																\
		_ch_->PropertyChange((PhidgetHandle)_ch_, _ch_->PropertyChangeCtx, (prop));					\
} while (0)

#define FIRE_ERROR(phid, ecode, ...)	do {														\
	PhidgetChannelHandle _ch_;																		\
	char _errbuf_[128];																				\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_ && _ch_->Error) {																		\
		snprintf(_errbuf_, sizeof(_errbuf_), __VA_ARGS__);											\
		_ch_->Error((PhidgetHandle)_ch_, _ch_->ErrorCtx, ecode, _errbuf_);							\
	}																								\
} while (0)

#define SEND_ERROR_EVENT(phid, ecode, ...)	do {													\
	PhidgetChannelHandle _ch_;																		\
	char _errbuf_[128];																				\
	_ch_ = PhidgetChannelCast(phid);																\
	if (_ch_) {																						\
		snprintf(_errbuf_, sizeof(_errbuf_), __VA_ARGS__);											\
		bridgeSendToChannel(_ch_, BP_ERROREVENT, "%d%s", ecode, _errbuf_);							\
	}																								\
} while (0)

#define DEVICECREATE_BODY(pname, pdef)											\
	Phidget##pname##Handle phid;												\
	TESTPTR(phidp);																\
	phid = mos_zalloc(sizeof (*phid));											\
	phidget_init((PhidgetHandle)phid, PHIDGET_DEVICE, 							\
		(PhidgetDelete_t)Phidget##pname##_free);								\
	phid->phid.deviceInfo.class = pdef;											\
	phid->phid.bridgeInput = Phidget##pname##_bridgeInput;						\
	phid->phid.initAfterOpen = Phidget##pname##_initAfterOpen;					\
	phid->phid.dataInput = Phidget##pname##_dataInput;							\
	*phidp = phid

#define CHANNELCREATE_BODY(pname, pdef)											\
	Phidget##pname##Handle phid;												\
	TESTPTR(phidp);																\
	phid = mos_zalloc(sizeof (*phid));											\
	phidget_init((PhidgetHandle)phid, PHIDGET_CHANNEL, 							\
		(PhidgetDelete_t)Phidget##pname##_free);								\
	phid->phid.class = pdef;													\
	phid->phid.initAfterOpen = Phidget##pname##_initAfterOpen;					\
	phid->phid.setDefaults = Phidget##pname##_setDefaults;						\
	phid->phid.fireInitialEvents = Phidget##pname##_fireInitialEvents;			\
	phid->phid.bridgeInput = Phidget##pname##_bridgeInput;						\
	phid->phid.errorHandler = Phidget##pname##_errorHandler;					\
	phid->phid.getStatus = Phidget##pname##_getStatus;							\
	phid->phid.setStatus = Phidget##pname##_setStatus;							\
	MTAILQ_INIT(&phid->phid.netconns);											\
	mos_mutex_init(&phid->phid.netconnslk);										\
	phid->phid.openInfo = mallocPhidgetOpenInfo();								\
	*phidp = phid

#define DEVBRIDGEINPUT(ch, bp)	deviceBridgeInput((ch), (bp), 0)

#define ASYNC_CALLBACK_ON_ERROR(stat)	do {									\
	PhidgetReturnCode res1 = (stat);											\
	if (res1 != EPHIDGET_OK && fptr)											\
		fptr((PhidgetHandle)phid, ctx, res1);									\
} while (0)

#define LOCALCALL(x)	do {													\
	if (!isNetworkPhidget(phid)) res = (x);										\
	else res = EPHIDGET_OK;														\
} while (0)

#define ISCHANNEL(phid)				(phid->type == PHIDGET_CHANNEL)
#define ISATTACHED(phid)			(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG) == PHIDGET_ATTACHED_FLAG)
#define ISATTACHEDORATTACHING(phid)	(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG) ? 1 : 0)
#define ISATTACHEDORDETACHING(phid)	(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_DETACHING_FLAG) ? 1 : 0)
#define ISATTACHEDDONE(phid)		(PhidgetCKFlags(phid, PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG) == PHIDGET_ATTACHED_FLAG)
#define ISOPEN(phid)				(PhidgetCKFlags(phid, PHIDGET_OPEN_FLAG) == PHIDGET_OPEN_FLAG)

#define _ISATTACHEDDONE(phid)			(((phid)->__flags & (PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG)) == PHIDGET_ATTACHED_FLAG)
#define _ISATTACHED(phid)				(((phid)->__flags & PHIDGET_ATTACHED_FLAG) == PHIDGET_ATTACHED_FLAG)
#define _ISATTACHEDORATTACHING(phid)	(((phid)->__flags & (PHIDGET_ATTACHED_FLAG | PHIDGET_ATTACHING_FLAG)) ? 1 : 0)
#define _ISOPEN(phid)					(((phid)->__flags & PHIDGET_OPEN_FLAG) == PHIDGET_OPEN_FLAG)
#define _ISDISPATCH(phid)				(((phid)->__flags & PHIDGET_DISPATCH_FLAG) == PHIDGET_DISPATCH_FLAG)
#define _ISDETACHING(phid)				(((phid)->__flags & PHIDGET_DETACHING_FLAG) == PHIDGET_DETACHING_FLAG)
#define _ISATTACHING(phid)				(((phid)->__flags & PHIDGET_ATTACHING_FLAG) == PHIDGET_ATTACHING_FLAG)

#define GETDEVICE(device, phid) do {											\
	device = PhidgetDeviceCast(phid);											\
	if (device == NULL)															\
		device = getParent(phid);												\
	else																		\
		PhidgetRetain(device);													\
} while (0);

#define CHANNELNOTDEVICE(ch, phid) do {				\
	PhidgetDeviceHandle __devh__;					\
	if (phid == NULL)								\
		return (EPHIDGET_INVALIDARG);					\
	(ch) = PhidgetChannelCast((phid));				\
	if ((ch) == NULL) {								\
		__devh__ = PhidgetDeviceCast(phid);			\
		if (__devh__ != NULL)						\
			return (EPHIDGET_UNSUPPORTED);			\
		return (EPHIDGET_INVALIDARG);					\
	}												\
} while (0)

#endif
