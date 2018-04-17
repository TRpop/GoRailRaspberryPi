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

#include "phidgetbase.h"
#include "mos/mos_os.h"
#include "mos/bsdqueue.h"

#include "network/network.h"

#include "manager.h"
#include "phidget.h"

#if defined(_MACOSX) && !defined(_IPHONE)
#include "macusb.h"
#endif

void FormatInit(void);
void PhidgetDispatchInit(void);
void PhidgetDispatchFini(void);
void PhidgetDispatchStop(void);
void PhidgetObjectInit(void);
void PhidgetObjectFini(void);
void PhidgetStatsInit(void);
void PhidgetStatsFini(void);

static void joinCentralThread(void);

static MOS_TASK_RESULT CentralThreadFunction(void *);
static mos_task_t centralThread;
static int centralThreadRun;
static mos_mutex_t centralThreadLock;
static mos_cond_t centralThreadCond;

/* Called by MOS_PANIC to log reason */
void
mos_log_err(const char *fmt, ...) {
	char buf[256];
	va_list va;

	va_start(va, fmt);
	mos_vsnprintf(buf, sizeof (buf), fmt, va);
	va_end(va);

	fprintf(stderr, "%s\n", buf);
	logcrit("%s", buf);
}

#ifdef _MACOSX
static CFRunLoopRef runLoop;
static int macInitDone = PFALSE;
#endif

static void
_Phidget22Initialize() {
	mos_mutex_init(&centralThreadLock);
	mos_cond_init(&centralThreadCond);
}

#ifdef _WINDOWS
#define _CONSTRUCTOR_
#define _DESTRUCTOR_
#else
#if defined(__GNUC__) || defined(__clang__)
#define _CONSTRUCTOR_ __attribute__((constructor))
#define _DESTRUCTOR_ __attribute__((destructor))
#else
#error "compiler does not support constructor/destructor"
#endif
#endif /* !_WINDOWS */

static int
mallocprint(void *arg, const char *fmt, ...) {
	char buf[1024];
	va_list va;
	int res;

	va_start(va, fmt);
	res = mos_raw_vsnprintf(buf, sizeof(buf), fmt, va);
	va_end(va);

	if (res < 0 || res > (int)sizeof (buf) - 1)
		buf[sizeof (buf) - 1] = '\0';

#ifdef _WINDOWS
	OutputDebugStringA(buf);
#else
	fprintf(stderr, "%s", buf);
#endif

	return (0);
}

mos_malloc_printer_t mallocprinter = mallocprint;

_CONSTRUCTOR_
static void
Phidget22Initialize() {

	mos_init();
	FormatInit();
	PhidgetStatsInit();
	PhidgetLogInit();
	PhidgetObjectInit();
	_Phidget22Initialize();
	PhidgetManagerInit();
	PhidgetInit();
	PhidgetDispatchInit();
	PhidgetNetInit();
}

//_DESTRUCTOR_
static void
Phidget22Finalize() {

	joinCentralThread();

	PhidgetDispatchFini();
	PhidgetNetFini();
	PhidgetManagerFini();
	PhidgetFini();
	PhidgetObjectFini();
	PhidgetLogFini();
	PhidgetStatsFini();

#ifdef MOS_TRACK_ALLOCATIONS
	if (mos_print_allocated_bytes())
		mos_dump_allocation_set(0, mallocprinter, NULL);
#endif

	mos_mutex_destroy(&centralThreadLock);
	mos_cond_destroy(&centralThreadCond);

	mos_fini();
}

PhidgetReturnCode
Phidget_finalize(int flags) {

	Phidget22Finalize();
	return (EPHIDGET_OK);
}

#ifdef _WINDOWS
#ifdef _MANAGED
#pragma managed(push, off)
#endif

BOOL APIENTRY
DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {

	switch (ul_reason_for_call) {
	case DLL_PROCESS_ATTACH:
		Phidget22Initialize();
		break;
	case DLL_THREAD_ATTACH:
		break;
	case DLL_THREAD_DETACH:
		break;
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#ifdef _MANAGED
#pragma managed(pop)
#endif
#endif


#ifdef _MACOSX
void macPeriodicTimerFunction(CFRunLoopTimerRef timer, void *Handle);
void macFindOpenChannelsSource(void *nothing);
CFRunLoopTimerRef timer = NULL;
CFRunLoopSourceRef findOpenChannelsSource = NULL;
#endif

void
NotifyCentralThread() {

	mos_cond_broadcast(&centralThreadCond);
}

PhidgetReturnCode
StartCentralThread() {
	PhidgetReturnCode res;

	mos_mutex_lock(&centralThreadLock);
	if (centralThreadRun == 1) {
		mos_cond_broadcast(&centralThreadCond);
		mos_mutex_unlock(&centralThreadLock);
		return (EPHIDGET_OK);
	}

#ifdef _MACOSX
	if (findOpenChannelsSource == NULL) {
		CFRunLoopSourceContext sourceContext = { 0 };
		sourceContext.perform = macFindOpenChannelsSource;
		findOpenChannelsSource = CFRunLoopSourceCreate(kCFAllocatorDefault, 0, &sourceContext);
	}
	if (timer == NULL)
		timer = CFRunLoopTimerCreate(kCFAllocatorDefault, 0, 0.250, 0, 0, macPeriodicTimerFunction, NULL);
#endif

	centralThreadRun = 1;
	res = mos_task_create(&centralThread, CentralThreadFunction, NULL);
	if (res != 0) {
		centralThreadRun = 0;
		mos_mutex_unlock(&centralThreadLock);
		return (res);
	}

	mos_mutex_unlock(&centralThreadLock);

#ifdef _MACOSX
		//make sure mac thread stuff is initialized
		while (!macInitDone)
			mos_usleep(10000);
		//run findOpenChannels in the context of the central thread
		CFRunLoopSourceSignal(findOpenChannelsSource);
		CFRunLoopWakeUp(runLoop);
#endif

	return (EPHIDGET_OK);
}

static int
needCentralThread() {

	if (centralThreadRun != 1)
		return (0);

	if (CHANNELS_EMPTY && MTAILQ_EMPTY(&phidgetManagerList) && !PhidgetNet_isStarted())
		return (0);
	return (1);
}

static void
joinCentralThread() {

#ifdef _MACOSX
		PhidgetUSBTeardownNotifications();

#ifdef LIGHTNING_SUPPORT
		PhidgetLightning_teardown();
#endif

		CFRunLoopRemoveTimer(runLoop, timer, kCFRunLoopDefaultMode);
		CFRunLoopRemoveSource(runLoop, findOpenChannelsSource, kCFRunLoopDefaultMode);

		CFRunLoopStop(runLoop);

		macInitDone = PFALSE;
#endif

	mos_mutex_lock(&centralThreadLock);
	while (centralThreadRun != 0) {
		centralThreadRun = 2;
		mos_cond_broadcast(&centralThreadCond);
		mos_cond_wait(&centralThreadCond, &centralThreadLock);
	}
	mos_mutex_unlock(&centralThreadLock);

}

#ifdef _MACOSX
//this is run every 250ms from the CentralThread runLoop
void
macPeriodicTimerFunction(CFRunLoopTimerRef timer, void *Handle) {
	PhidgetDeviceHandle device;

	logverbose("running macPeriodicTimerFunction()");
	PhidgetLockDevices();

	FOREACH_DEVICE(device) {
		switch (device->deviceInfo.class) {
			case PHIDCLASS_MESHDONGLE:
				addMeshHubs(device);
				break;
			case PHIDCLASS_HUB:
				scanVintDevices(device);
				break;
			default:
				break;
		}
	}

	PhidgetUnlockDevices();

	mos_mutex_lock(&centralThreadLock);
	if (needCentralThread()) {
		matchOpenChannels(); //this looks for attached active devices and opens them
	}
	mos_mutex_unlock(&centralThreadLock);

	return;
}

//This is run when a new Phidget is registered when the CentralThread is already running
void
macFindOpenChannelsSource(void *ctx) {
	logverbose("running macFindOpenChannelsSource()");
	mos_mutex_lock(&centralThreadLock);
	if (needCentralThread()) {
		matchOpenChannels(); //this looks for attached active devices and opens them
	}
	mos_mutex_unlock(&centralThreadLock);
	return;
}
#endif

//The central thread should stop itself when there are no more active devices...?
//Or we can stop it in unregisterlocaldevice

static MOS_TASK_RESULT
CentralThreadFunction(void *_param) {
	PhidgetDeviceHandle device;

	logverbose("Central Thread running");

#ifdef SPI_SUPPORT
	populateAttachedSPIDevices();
#endif

#ifdef _MACOSX
	PhidgetReturnCode res;

	runLoop = CFRunLoopGetCurrent();

	CFRunLoopAddTimer(runLoop, timer, kCFRunLoopDefaultMode);
	CFRunLoopAddSource(runLoop, findOpenChannelsSource, kCFRunLoopDefaultMode);

	macInitDone = PTRUE;

	//setup notifications of Phidget attach/detach
	res = PhidgetUSBSetupNotifications(runLoop);
	if (res != EPHIDGET_OK) {
		logerr("PhidgetUSBSetupNotifications failed: %d", res);
	}

#ifdef LIGHTNING_SUPPORT
	PhidgetLightning_setup();
#endif

	//start run loop - note that this blocks until JoinCentralThread() is called.
	logverbose("Starting CFRunLoopRun()...");
	CFRunLoopRun();
	mos_mutex_lock(&centralThreadLock);

#else

	mos_mutex_lock(&centralThreadLock);
	while (needCentralThread()) {
		PhidgetManager_poll();
		matchOpenChannels();

		mos_cond_timedwait(&centralThreadCond, &centralThreadLock, 250000000);
	}
#endif

#ifdef SPI_SUPPORT
	clearAttachedSPIDevices();
#endif

	PhidgetLockDevices();
	while ((device = MTAILQ_FIRST(&phidgetDevices)) != NULL) {
		chlog("centralthread %"PRIphid"", device);
		deviceDetach(device, 0);
	}
	MOS_ASSERT(DEVICES_EMPTY);
	MTAILQ_INIT(&phidgetDevices);
	PhidgetUnlockDevices();

	/* Shut down all dispatchers */
	PhidgetDispatchStop();

#if defined(_LINUX) || defined(_FREEBSD) && !defined(_ANDROID)
	//Shut down USB
	PhidgetUSBUninit();
#endif

	logverbose("Central Thread exiting");

	centralThreadRun = 0;
	mos_cond_broadcast(&centralThreadCond);
	mos_mutex_unlock(&centralThreadLock);

	MOS_TASK_EXIT(EPHIDGET_OK);
}

API_PRETURN
Phidget_resetLibrary() {
	PhidgetChannelHandle channel, channeltmp;
	PhidgetManagerHandle manager, managertmp;

	channel = MTAILQ_FIRST(&phidgetChannels);
	while (channel != NULL) {
		channeltmp = MTAILQ_NEXT(channel, link);
		Phidget_close((PhidgetHandle)channel);
		PhidgetRelease(&channel);
		channel = channeltmp;
	}

	manager = MTAILQ_FIRST(&phidgetManagerList);
	while (manager != NULL) {
		managertmp = MTAILQ_NEXT(manager, link);
		PhidgetManager_close(manager);
		PhidgetManager_delete(&manager);
		manager = managertmp;
	}

	Phidget22Finalize();
	Phidget22Initialize();

	return (EPHIDGET_OK);
}
