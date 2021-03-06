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
#include "gpp.h"
#include "device/spatialdevice.h"

// === Internal Functions === //
static double getCorrectedField(PhidgetSpatialDeviceHandle phid, double fields[], int axis);
static PhidgetReturnCode PhidgetSpatialDevice_zeroGyro(PhidgetSpatialDeviceHandle phid);
static PhidgetReturnCode PhidgetSpatialDevice_setDataRate(PhidgetSpatialDeviceHandle phid, int milliseconds);
static PhidgetReturnCode PhidgetSpatialDevice_setCompassCorrectionParameters(PhidgetSpatialDeviceHandle phid, double magField, double offset0, double offset1, double offset2, double gain0, double gain1, double gain2, double T0, double T1, double T2, double T3, double T4, double T5);
static PhidgetReturnCode PhidgetSpatialDevice_resetCompassCorrectionParameters(PhidgetSpatialDeviceHandle phid);
static PhidgetReturnCode PhidgetSpatialDevice_saveCompassCorrectionParameters(PhidgetSpatialDeviceHandle phid);

//initAfterOpen - sets up the initial state of an object, reading in packets from the device if needed
//				  used during attach initialization - on every attach
static PhidgetReturnCode CCONV
PhidgetSpatialDevice_initAfterOpen(PhidgetDeviceHandle device) {
	PhidgetSpatialDeviceHandle phid = (PhidgetSpatialDeviceHandle)device;
	int i = 0;

	assert(phid);

	//Setup max/min values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1049:
		phid->accelerationMax = 5;
		phid->accelerationMin = -5;
		phid->interruptRate = 8;
		phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
		phid->dataInterval[0] = phid->interruptRate;
		phid->dataRateMax = 1; //actual data rate
		phid->angularRateMax = 0;
		phid->angularRateMin = 0;
		phid->magneticFieldMax = 0;
		phid->magneticFieldMin = 0;
		phid->calDataValid = PFALSE;
		break;

	case PHIDUID_1041:
	case PHIDUID_1043:
		phid->accelerationMax = 8;
		phid->accelerationMin = -8;
		phid->interruptRate = 8;
		phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
		phid->dataInterval[0] = phid->interruptRate;
		phid->dataRateMax = 1; //actual data rate
		phid->angularRateMax = 0;
		phid->angularRateMin = 0;
		phid->magneticFieldMax = 0;
		phid->magneticFieldMin = 0;
		phid->calDataValid = PFALSE;
		break;

	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:
		phid->accelerationMax = 5;
		phid->accelerationMin = -5;
		phid->interruptRate = 8;
		phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
		phid->dataInterval[0] = phid->interruptRate;
		phid->dataRateMax = 4; //actual data rate
		phid->angularRateMax = 400;
		phid->angularRateMin = -400;
		phid->magneticFieldMax = 4;
		phid->magneticFieldMin = -4;
		phid->userMagField = 1.0;
		phid->calDataValid = PFALSE;
		break;

	case PHIDUID_1042:
	case PHIDUID_1044:
		phid->accelerationMax = 8;
		phid->accelerationMin = -8;
		phid->interruptRate = 4;
		phid->dataRateMin = SPATIAL_MIN_DATA_RATE;
		phid->dataInterval[0] = 8;
		phid->dataRateMax = 4; //actual data rate
		phid->angularRateMax = 2000;
		phid->angularRateMin = -2000;
		phid->magneticFieldMax = 5.6;
		phid->magneticFieldMin = -5.6;
		phid->userMagField = 1.0;
		phid->calDataValid = PFALSE;
		break;

	default:
		MOS_PANIC("Unexpected device");
	}

	//initialize triggers, set data arrays to unknown
	for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
		phid->acceleration[0][i] = PUNK_DBL;
		phid->accelGain1[i] = PUNK_DBL;
		phid->accelGain2[i] = PUNK_DBL;
		phid->accelOffset[i] = PUNK_DBL;
	}
	phid->accelerationChangeTrigger[0] = 0;
	for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
		phid->angularRate[0][i] = PUNK_DBL;
		phid->gryoCorrection[i] = 0;
		phid->gyroGain1[i] = PUNK_DBL;
		phid->gyroGain2[i] = PUNK_DBL;
		phid->gyroOffset[i] = PUNK_DBL;
	}
	for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
		phid->magneticField[0][i] = PUNK_DBL;
		phid->userCompassGain[i] = 1.0;
	}
	phid->magneticFieldChangeTrigger[0] = 0;
	phid->bufferReadPtr = 0;
	phid->bufferWritePtr = 0;
	phid->timestamp[0] = 0;
	phid->lastEventTime = 0;
	phid->latestDataTime = 0;

	phid->lastTimeCounterValid = PFALSE;
	phid->doZeroGyro = PFALSE;

	//get calibration values
	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1049:
	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:
	{
		uint8_t buffer[1];
		PhidgetReturnCode result;
		int readCount = 125; // up to 1 second of data - should be PLENTY
		//ask for calibration values
		buffer[0] = SPATIAL_READCALIB;

		result = PhidgetDevice_sendpacket((PhidgetDeviceHandle)phid, buffer, 1);
		if (result != EPHIDGET_OK)
			return result;

		while (phid->calDataValid == PFALSE && readCount--) {
			//note that Windows queues up to 32 packets, so we need to read at least this many to get the calibration packet
			waitForReads((PhidgetDeviceHandle)phid, 1, 100);
		}
		if (!phid->calDataValid)
			return (EPHIDGET_UNEXPECTED);
	}
	break;
// No streamed Calibration
	case PHIDUID_1041:
	case PHIDUID_1043:
	case PHIDUID_1042:
	case PHIDUID_1044:
	default:
		break;
	}

	//issue one read
	//this should fill in the data because the dataRate is the interrupt rate
	waitForReads((PhidgetDeviceHandle)phid, 1, 100);

	return (EPHIDGET_OK);
}

static void
updateTimestamp(PhidgetSpatialDeviceHandle phid, int time) {
	if (phid->lastTimeCounterValid) {
		//1-255 ms
		int timechange = (uint16_t)((uint16_t)time - (uint16_t)phid->lastTimeCounter);
		phid->timestamp[0] += timechange;
	} else {
		phid->lastTimeCounterValid = PTRUE;
	}
	phid->lastTimeCounter = time;
}

static void
updateLatestDataTime(PhidgetSpatialDeviceHandle phid, int i) {
	phid->latestDataTime = phid->timestamp[0] + (i + 1) * phid->dataRateMax;
}

static double
timestampdiff(double time1, double time2) {
	return (time1 - time2);
}

//dataInput - parses device packets
static PhidgetReturnCode CCONV
PhidgetSpatialDevice_dataInput(PhidgetDeviceHandle device, uint8_t *buffer, size_t length) {
	PhidgetSpatialDeviceHandle phid = (PhidgetSpatialDeviceHandle)device;
	PhidgetChannelHandle channel;
	int i = 0, j = 0, count = 0, cal;
	uint8_t doneGyroZero = PFALSE;
	double accelAvg[SPATIAL_MAX_ACCELAXES] = { 0 };
	double angularRateAvg[SPATIAL_MAX_ACCELAXES] = { 0 };
	double magneticFieldAvg[SPATIAL_MAX_ACCELAXES] = { 0 };
	double magneticFieldCorr[SPATIAL_MAX_ACCELAXES];
	PhidgetSpatialDevice_SpatialDeviceEventData eventData[16];
	uint64_t dataRate;
	int fireSaturation;
	int fireEvent;

	dataRate = phid->dataInterval[0];

	assert(phid);
	assert(buffer);

	//Parse device packets - store data locally
	switch (device->deviceInfo.UDD->uid) {
	case PHIDUID_1049:
	{
		int data;
		double accelUncalib[3] = { 0,0,0 };
		int time;

		//top 2 bits in buffer[0] are packet type
		switch (buffer[0] & 0xc0) {
		case SPATIAL_PACKET_DATA:
			if (phid->calDataValid) {
				count = buffer[0] / 3;
				if (count == 0)
					goto done;

				//this timestamp is for the latest data
				time = ((uint16_t)buffer[1] << 8) + (uint16_t)buffer[2];
				if (phid->lastTimeCounterValid) {
					//0-255 ms
					int timechange = (uint16_t)((uint16_t)time - (uint16_t)phid->lastTimeCounter);
					phid->timestamp[0] += timechange;
				} else {
					phid->lastTimeCounterValid = PTRUE;
				}
				phid->lastTimeCounter = time;

				//add data to data buffer
				for (i = 0; i < count; i++) {
					//LIS344ALH - Vdd/15 V/g - 0x1fff/15 = 0x222 (546.06666666666666666666666666667)
					for (j = 0; j < 3; j++) {
						data = ((uint16_t)buffer[3 + j * 2 + i * 6] << 8) + (uint16_t)buffer[4 + j * 2 + i * 6];
						accelUncalib[j] = ((double)data - 0x0fff) / 546.066667;
					}
					accelUncalib[1] = -accelUncalib[1]; //reverse Y-axis
					//Apply offsets
					for (j = 0; j < 3; j++) {
						accelUncalib[j] -= phid->accelOffset[j];
					}
					//X
					if (accelUncalib[0] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = accelUncalib[0] * phid->accelGain1[0] + accelUncalib[1] * phid->accelFactor1[0] + accelUncalib[2] * phid->accelFactor2[0];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = accelUncalib[0] * phid->accelGain2[0] + accelUncalib[1] * phid->accelFactor1[0] + accelUncalib[2] * phid->accelFactor2[0];
					//Y
					if (accelUncalib[1] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = accelUncalib[1] * phid->accelGain1[1] + accelUncalib[0] * phid->accelFactor1[1] + accelUncalib[2] * phid->accelFactor2[1];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = accelUncalib[1] * phid->accelGain2[1] + accelUncalib[0] * phid->accelFactor1[1] + accelUncalib[2] * phid->accelFactor2[1];
					//Z
					if (accelUncalib[2] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = accelUncalib[2] * phid->accelGain1[2] + accelUncalib[0] * phid->accelFactor1[2] + accelUncalib[1] * phid->accelFactor2[2];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = accelUncalib[2] * phid->accelGain2[2] + accelUncalib[0] * phid->accelFactor1[2] + accelUncalib[1] * phid->accelFactor2[2];

					updateLatestDataTime(phid, i);

					phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

					phid->bufferWritePtr++;
					if (phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
						phid->bufferWritePtr = 0;
				}
			}
			break;
		case SPATIAL_PACKET_CALIB:
			for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
				cal = ((uint16_t)buffer[i * 7 + 1] << 4) + ((uint16_t)buffer[i * 7 + 2] >> 4);
				phid->accelGain1[i] = cal / (4096 / 0.4) + 0.8;
				cal = (((uint16_t)buffer[i * 7 + 2] << 8) & 0x0F00) | ((uint16_t)buffer[i * 7 + 3]);
				phid->accelGain2[i] = cal / (4096 / 0.4) + 0.8;
				cal = (uint16_t)((uint16_t)buffer[i * 7 + 4] << 8) + (uint16_t)buffer[i * 7 + 5];
				phid->accelOffset[i] = cal / (65535 / 1.0) - 0.5;
				cal = (uint8_t)buffer[i * 7 + 6];
				phid->accelFactor1[i] = cal / (256 / 0.2) - 0.1;
				cal = (uint8_t)buffer[i * 7 + 7];
				phid->accelFactor2[i] = cal / (256 / 0.2) - 0.1;
				//logdebug("Accel(%d) Calib: %1.4lf, %1.4lf, %1.4lf, %1.4lf, %1.4lf", i,
				//	phid->accelGain1[i], phid->accelGain2[i], phid->accelOffset[i],
				//  phid->accelFactor1[i], phid->accelFactor2[i]);
			}
			phid->calDataValid = PTRUE;
			break;
		}
		break;
	}
	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:
		//top 2 bits in buffer[0] are packet type
		switch (buffer[0]) {
		case SPATIAL_PACKET_DATA:
			if (phid->calDataValid) {
				int data;
				double accelUncalib[3] = { 0,0,0 };
				double gyroUncalib[3] = { 0,0,0 };
				int time;

				count = (buffer[1] & 0x1f) / 9;
				if (count == 0)
					goto done;

				//this timestamp is for the latest data
				time = ((uint16_t)buffer[2] << 8) + (uint16_t)buffer[3];
				if (phid->lastTimeCounterValid) {
					//0-255 ms
					int timechange = (uint16_t)((uint16_t)time - (uint16_t)phid->lastTimeCounter);
					phid->timestamp[0] += timechange;
				} else {
					phid->lastTimeCounterValid = PTRUE;
				}
				phid->lastTimeCounter = time;

				//add data to data buffer
				for (i = 0; i < count; i++) {
					//LIS344ALH - Vdd/15 V/g - 0xffff/15 = 0x1111 (4369.0)
					for (j = 0; j < 3; j++) {
						data = ((uint16_t)buffer[4 + j * 2 + i * 18] << 8) + (uint16_t)buffer[5 + j * 2 + i * 18];
						accelUncalib[j] = ((double)data - 0x7fff) / 4369.0;
					}
					accelUncalib[1] = -accelUncalib[1]; //reverse Y-axis
					//Apply offsets
					for (j = 0; j < 3; j++) {
						accelUncalib[j] -= phid->accelOffset[j];
					}
					//X
					if (accelUncalib[0] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = accelUncalib[0] * phid->accelGain1[0] + accelUncalib[1] * phid->accelFactor1[0] + accelUncalib[2] * phid->accelFactor2[0];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[0] = accelUncalib[0] * phid->accelGain2[0] + accelUncalib[1] * phid->accelFactor1[0] + accelUncalib[2] * phid->accelFactor2[0];
					//Y
					if (accelUncalib[1] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = accelUncalib[1] * phid->accelGain1[1] + accelUncalib[0] * phid->accelFactor1[1] + accelUncalib[2] * phid->accelFactor2[1];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[1] = accelUncalib[1] * phid->accelGain2[1] + accelUncalib[0] * phid->accelFactor1[1] + accelUncalib[2] * phid->accelFactor2[1];
					//Z
					if (accelUncalib[2] > 0)
						phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = accelUncalib[2] * phid->accelGain1[2] + accelUncalib[0] * phid->accelFactor1[2] + accelUncalib[1] * phid->accelFactor2[2];
					else
						phid->dataBuffer[phid->bufferWritePtr].acceleration[2] = accelUncalib[2] * phid->accelGain2[2] + accelUncalib[0] * phid->accelFactor1[2] + accelUncalib[1] * phid->accelFactor2[2];

					//ADC ref is 0-3.3V - 50.355uV/bit, gyro zero rate is 1.23V, 2.5mV/deg/s - these voltages are fixed, non-ratiometric to Vref
					// 1 / 0.000050355 = 19859 (1V)
					// 1.23 * 19859 = 24427
					// 0.0025 * 19859 = 49.6477bits/deg/s
					for (j = 0; j < 3; j++) {
						data = ((uint16_t)buffer[10 + j * 2 + i * 18] << 8) + (uint16_t)buffer[11 + j * 2 + i * 18];
						if (j == 1)
							gyroUncalib[j] = ((double)(data - 24427) + phid->gyroOffset[j]) / 49.6477;
						else
							gyroUncalib[j] = ((double)-(data - 24427) + phid->gyroOffset[j]) / 49.6477; //reverse X/Z-axis
					}
					//0 - we multiply these by their gains so revered axes will still appear positive and get the correct gain
					if ((gyroUncalib[0] * phid->gyroGain1[0]) > 0)
						phid->dataBuffer[phid->bufferWritePtr].angularRate[0] = gyroUncalib[0] * phid->gyroGain1[0] - gyroUncalib[1] * phid->gyroFactor1[0] - gyroUncalib[2] * phid->gyroFactor2[0];
					else
						phid->dataBuffer[phid->bufferWritePtr].angularRate[0] = gyroUncalib[0] * phid->gyroGain2[0] - gyroUncalib[1] * phid->gyroFactor1[0] - gyroUncalib[2] * phid->gyroFactor2[0];
					//1
					if ((gyroUncalib[1] * phid->gyroGain1[1]) > 0)
						phid->dataBuffer[phid->bufferWritePtr].angularRate[1] = gyroUncalib[1] * phid->gyroGain1[1] - gyroUncalib[0] * phid->gyroFactor1[1] - gyroUncalib[2] * phid->gyroFactor2[1];
					else
						phid->dataBuffer[phid->bufferWritePtr].angularRate[1] = gyroUncalib[1] * phid->gyroGain2[1] - gyroUncalib[0] * phid->gyroFactor1[1] - gyroUncalib[2] * phid->gyroFactor2[1];
					//2
					if ((gyroUncalib[2] * phid->gyroGain1[2]) > 0)
						phid->dataBuffer[phid->bufferWritePtr].angularRate[2] = gyroUncalib[2] * phid->gyroGain1[2] - gyroUncalib[0] * phid->gyroFactor1[2] - gyroUncalib[1] * phid->gyroFactor2[2];
					else
						phid->dataBuffer[phid->bufferWritePtr].angularRate[2] = gyroUncalib[2] * phid->gyroGain2[2] - gyroUncalib[0] * phid->gyroFactor1[2] - gyroUncalib[1] * phid->gyroFactor2[2];

					//checks if compass data is valid
					//Note: we miss ~7 samples (28ms) every second while the compass is calibrating
					if (buffer[1] & (0x80 >> i)) {
						//ADC 50.355uV/bit (0-3.3V)
						//ideal compass midpoint is 0x7FFF (32767) (1.65V)
						//valid range for zero field offset is: 0.825V - 2.475V (16384-49151) (+-16384)
						// Note that this may be less (~3x) because the Gain is less, but I'm not sure. (+-5460)
						//valid output voltage range is defined as 0.165V - 3.135V (3277-62258),
						// so we can't really trust values outside of this, though we do seem to get valid data...
						//ideal sensitivity is 250mV/gauss (ext. resistor), valid range is 195 - 305
						// 1 / 0.000050355 = 19859 (1Volt)
						// 0.250 * 19859 = 4964.75 bits/gauss (1.0 gain) (ideal) - valid range is (3861-6068) (+-1103)
						//We have defined the compass gain multiplier to be based on 6500bits/gauss to keep the math resonable,
						// so we must use that value here. Implications?
						//The largest range we can guarantee is:
						// 16384-3277/6068 = +-2.16 gauss or, more likely: +-3.96 gauss
						// Ideal is: 32767-3277/4964.75 = +-5.94 gauss
						// we can tell from the incoming data whether it's valid or not,
						// we'll probably have more range in one dirrection then the other because of offset.
						for (j = 0; j < phid->devChannelCnts.numCompassAxes; j++) {
							data = ((uint16_t)buffer[16 + i * 18 + j * 2] << 8) + (uint16_t)buffer[17 + i * 18 + j * 2];
							//if we are not within (3277-62258), data is not valid
							if (data < 3277) {
								phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = phid->magneticFieldMin - 0.1;
								break;
							}
							if (data > 62258) {
								phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = phid->magneticFieldMax + 0.1;
								break;
							}

							//if gain or offset don't make sense, throw out data
							//if (phid->compassGain[j] > 6068 || phid->compassGain[j] < 3861 ||
							if (phid->compassGain[j] > 6068 || phid->compassGain[j] < 2500 || //lower gains seem to be common
								phid->compassOffset[j] > 5460 || phid->compassOffset[j] < -5460) {
								if (data > 32767)
									phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = phid->magneticFieldMax + 0.1;
								else
									phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = phid->magneticFieldMin - 0.1;
								break;
							}

							//Convert ADC to Gauss
							phid->dataBuffer[phid->bufferWritePtr].magneticField[j] =
								-((double)data - 0x7fff - phid->compassOffset[j]) / phid->compassGain[j];
						}

					} else {
						phid->dataBuffer[phid->bufferWritePtr].magneticField[0] = PUNK_DBL;
						phid->dataBuffer[phid->bufferWritePtr].magneticField[1] = PUNK_DBL;
						phid->dataBuffer[phid->bufferWritePtr].magneticField[2] = PUNK_DBL;
					}

					updateLatestDataTime(phid, i);

					phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

					phid->bufferWritePtr++;
					if (phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
						phid->bufferWritePtr = 0;
				}
			}
			break;
		case SPATIAL_PACKET_CALIB:
			for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
				cal = ((uint16_t)buffer[i * 7 + 1] << 4) + ((uint16_t)buffer[i * 7 + 2] >> 4);
				phid->accelGain1[i] = cal / (4096 / 0.4) + 0.8;
				cal = (((uint16_t)buffer[i * 7 + 2] << 8) & 0x0F00) | ((uint16_t)buffer[i * 7 + 3]);
				phid->accelGain2[i] = cal / (4096 / 0.4) + 0.8;
				cal = (uint16_t)((uint16_t)buffer[i * 7 + 4] << 8) + (uint16_t)buffer[i * 7 + 5];
				phid->accelOffset[i] = cal / (65535 / 1.0) - 0.5;
				cal = (uint8_t)buffer[i * 7 + 6];
				phid->accelFactor1[i] = cal / (256 / 0.2) - 0.1;
				cal = (uint8_t)buffer[i * 7 + 7];
				phid->accelFactor2[i] = cal / (256 / 0.2) - 0.1;
				//logdebug("Accel(%d) Calib: %1.4lf, %1.4lf, %1.4lf, %1.4lf, %1.4lf", i,
				//	phid->accelGain1[i], phid->accelGain2[i], phid->accelOffset[i],
				//  phid->accelFactor1[i], phid->accelFactor2[i]);
			}
			for (j = 0; j < phid->devChannelCnts.numGyroAxes; i++, j++) {
				if (device->deviceInfo.UDD->uid == PHIDUID_1056) {
					cal = ((uint16_t)buffer[i * 7 + 1] << 4) + ((uint16_t)buffer[i * 7 + 2] >> 4);
					phid->gyroGain1[j] = cal / (4096 / 0.4) + 0.8;
					cal = (((uint16_t)buffer[i * 7 + 2] << 8) & 0x0F00) | ((uint16_t)buffer[i * 7 + 3]);
					phid->gyroGain2[j] = cal / (4096 / 0.4) + 0.8;
				}
				//Allow for negative gains
				else if (device->deviceInfo.UDD->uid == PHIDUID_1056_NEG_GAIN) {
					cal = ((int16_t)((int8_t)buffer[i * 7 + 1]) << 4) + ((uint16_t)buffer[i * 7 + 2] >> 4);
					phid->gyroGain1[j] = cal / (4096 / 0.4) + (cal > 0 ? 0.9 : -0.9);
					cal = ((((uint16_t)buffer[i * 7 + 2] << 8) & 0x0F00) | ((uint16_t)buffer[i * 7 + 3])) << 4;
					cal = (int16_t)cal >> 4;
					phid->gyroGain2[j] = cal / (4096 / 0.4) + (cal > 0 ? 0.9 : -0.9);
				}
				cal = (int16_t)((uint16_t)buffer[i * 7 + 4] << 8) + (uint16_t)buffer[i * 7 + 5];
				phid->gyroOffset[j] = cal;
				cal = (uint8_t)buffer[i * 7 + 6];
				phid->gyroFactor1[j] = cal / (256 / 0.1) - 0.05;
				cal = (uint8_t)buffer[i * 7 + 7];
				phid->gyroFactor2[j] = cal / (256 / 0.1) - 0.05;

//Zero out calibrations
#if 0
				phid->gyroGain1[j] = 1;
				phid->gyroGain2[j] = 1;
				phid->gyroOffset[j] = 0;
				phid->gyroFactor1[j] = 0;
				phid->gyroFactor2[j] = 0;
#endif

				logverbose("Gyro(%d) Calib: %1.4lf, %1.4lf, %1.4lf, %1.4lf, %1.4lf", j,
					phid->gyroGain1[j], phid->gyroGain2[j], phid->gyroOffset[j],
					phid->gyroFactor1[j], phid->gyroFactor2[j]);
			}
			for (j = 0; j < phid->devChannelCnts.numCompassAxes; j++) {
				phid->compassOffset[j] = (int16_t)((uint16_t)buffer[j * 4 + 49] << 8) + (uint16_t)buffer[j * 4 + 50];
				phid->compassGain[j] = ((uint16_t)buffer[j * 4 + 51] << 8) + (uint16_t)buffer[j * 4 + 52];
				//phid->compassGain[j] = 4964;
			}
			//logdebug("Compass Gain: %d, %d, %d", phid->compassGain[0], phid->compassGain[1],
			//  phid->compassGain[2]);
			//logdebug("Compass Offset: %d, %d, %d", phid->compassOffset[0],
			//  phid->compassOffset[1], phid->compassOffset[2]);
			phid->calDataValid = PTRUE;
			break;
		}
		break;
	case PHIDUID_1041:
	case PHIDUID_1043:
	{
		int time;
		int analogOrDigital = ((uint16_t)buffer[1] << 8) + (uint16_t)buffer[2];
		count = buffer[0];

		if (count == 0)
			goto done;

		//this timestamp is for the latest data
		time = ((uint16_t)buffer[3] << 8) + (uint16_t)buffer[4];
		updateTimestamp(phid, time);

		//add data to data buffer
		for (i = 0; i < count; i++) {
			int countOffset = i * 6; //Each set of samples is 6 bytes
			for (j = 0; j < 3; j++) {
				int indexOffset = j * 2; //Each value is 2 bytes
				int16_t accelData = (int16_t)((uint16_t)buffer[5 + indexOffset + countOffset] << 8) + (uint16_t)buffer[6 + indexOffset + countOffset];

				//digital accel
				if (analogOrDigital & (0x01 << i))
					phid->dataBuffer[phid->bufferWritePtr].acceleration[j] = (double)accelData / SPATIAL_MMA8451Q_BITS_PER_G;
				//analog accel
				else
					phid->dataBuffer[phid->bufferWritePtr].acceleration[j] = (double)accelData / SPATIAL_KXR94_2050_w_AD7689_BITS_PER_G;
			}

			updateLatestDataTime(phid, i);

			phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

			phid->bufferWritePtr++;
			if (phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
				phid->bufferWritePtr = 0;
		}
		break;
	}
	case PHIDUID_1042:
	case PHIDUID_1044:
	{
		int time;
		int flags = buffer[1];
		count = buffer[0];

		if (count == 0)
			goto done;

		//this timestamp is for the latest data
		time = ((uint16_t)buffer[2] << 8) + (uint16_t)buffer[3];
		updateTimestamp(phid, time);

		//add data to data buffer
		for (i = 0; i < count; i++) {
			int countOffset = i * 18; //Each set of samples is 18 bytes
			for (j = 0; j < 3; j++) {
				int indexOffset = j * 2; //Each value is 2 bytes
				int16_t accelData = (int16_t)((uint16_t)buffer[4 + indexOffset + countOffset] << 8) + (uint16_t)buffer[5 + indexOffset + countOffset];
				int16_t gyroData = (int16_t)((uint16_t)buffer[10 + indexOffset + countOffset] << 8) + (uint16_t)buffer[11 + indexOffset + countOffset];
				int16_t magData = (int16_t)((uint16_t)buffer[16 + indexOffset + countOffset] << 8) + (uint16_t)buffer[17 + indexOffset + countOffset];

				//digital accel
				if (flags & (0x02 >> i))
					phid->dataBuffer[phid->bufferWritePtr].acceleration[j] = (double)accelData / SPATIAL_MMA8451Q_BITS_PER_G;
				//analog accel
				else
					phid->dataBuffer[phid->bufferWritePtr].acceleration[j] = (double)accelData / SPATIAL_KXR94_2050_w_AD7689_BITS_PER_G;

				//digital gyro
				if (flags & (0x08 >> i))
					phid->dataBuffer[phid->bufferWritePtr].angularRate[j] = (double)gyroData / SPATIAL_L3GD20_BITS_PER_DPS;
			//analog gyro
				else
					if (j == 2)
						phid->dataBuffer[phid->bufferWritePtr].angularRate[j] = (double)(gyroData) / SPATIAL_LY330ALH_w_AD7689_BITS_PER_DPS;
					else
						phid->dataBuffer[phid->bufferWritePtr].angularRate[j] = (double)(gyroData) / SPATIAL_LRP410AL_w_AD7689_BITS_PER_DPS;

				//compass valid
				if (flags & (0x20 >> i))
					phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = magData / SPATIAL_HMC5883L_BITS_PER_GAUSS;
				//no compass data
				else
					phid->dataBuffer[phid->bufferWritePtr].magneticField[j] = PUNK_DBL;
			}

			updateLatestDataTime(phid, i);

			phid->dataBuffer[phid->bufferWritePtr].timestamp = phid->latestDataTime;

			phid->bufferWritePtr++;
			if (phid->bufferWritePtr >= SPATIAL_DATA_BUFFER_SIZE)
				phid->bufferWritePtr = 0;
		}
		break;
	}
	default:
		MOS_PANIC("Unexpected device");
	}

	if (phid->doZeroGyro) {
		//done
		if (timestampdiff(phid->latestDataTime, phid->dataBuffer[phid->gyroZeroReadPtr].timestamp) >= SPATIAL_ZERO_GYRO_TIME) {
			double gryoCorrectionTemp[SPATIAL_MAX_GYROAXES] = { 0,0,0 };
			int gryoCorrectionCount = 0;

			while (phid->gyroZeroReadPtr != phid->bufferWritePtr) {
				for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
					gryoCorrectionTemp[i] += phid->dataBuffer[phid->gyroZeroReadPtr].angularRate[i];
				}

				phid->gyroZeroReadPtr++;
				if (phid->gyroZeroReadPtr >= SPATIAL_DATA_BUFFER_SIZE)
					phid->gyroZeroReadPtr = 0;

				gryoCorrectionCount++;
			}

			for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
				phid->gryoCorrection[i] = gryoCorrectionTemp[i] / (double)gryoCorrectionCount;
			}

			doneGyroZero = PTRUE;
		}
	}

	//see if it's time for an event
	if (timestampdiff(phid->latestDataTime, phid->lastEventTime) >= dataRate) {
		double tempTime;
		//int lastPtr;
		int accelCounter[SPATIAL_MAX_ACCELAXES] = { 0 };
		int angularRateCounter[SPATIAL_MAX_ACCELAXES] = { 0 };
		int magneticFieldCounter[SPATIAL_MAX_ACCELAXES] = { 0 };

		int dataPerEvent = 0;

		int multipleDataPerEvent = PFALSE;

		if (dataRate < (unsigned)phid->interruptRate)
			multipleDataPerEvent = PTRUE;

		for (j = 0;; j++) {
			//makes sure we read all data
			if (phid->bufferReadPtr == phid->bufferWritePtr || j >= 16) {
				dataPerEvent = j;
				break;
			}

			memset(accelCounter, 0, sizeof(accelCounter));
			memset(angularRateCounter, 0, sizeof(angularRateCounter));
			memset(magneticFieldCounter, 0, sizeof(magneticFieldCounter));

			tempTime = phid->dataBuffer[phid->bufferReadPtr].timestamp;

			//average data for each stage
			while (phid->bufferReadPtr != phid->bufferWritePtr &&
				(!multipleDataPerEvent || timestampdiff(phid->dataBuffer[phid->bufferReadPtr].timestamp, tempTime) < dataRate)) {
				for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
					if (phid->dataBuffer[phid->bufferReadPtr].acceleration[i] != PUNK_DBL) {
						accelAvg[i] += phid->dataBuffer[phid->bufferReadPtr].acceleration[i];
						accelCounter[i]++;
					}
				}
				for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
					if (phid->dataBuffer[phid->bufferReadPtr].angularRate[i] != PUNK_DBL) {
						double rate = phid->dataBuffer[phid->bufferReadPtr].angularRate[i] - phid->gryoCorrection[i];

						angularRateAvg[i] += rate;
						angularRateCounter[i]++;
					}
				}
				for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
					if (phid->dataBuffer[phid->bufferReadPtr].magneticField[i] != PUNK_DBL) {
						magneticFieldAvg[i] += phid->dataBuffer[phid->bufferReadPtr].magneticField[i];
						magneticFieldCounter[i]++;
					}
				}

				//lastPtr = phid->bufferReadPtr;

				phid->bufferReadPtr++;
				if (phid->bufferReadPtr >= SPATIAL_DATA_BUFFER_SIZE)
					phid->bufferReadPtr = 0;
			}

			for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
				if (accelCounter[i] > 0)
					eventData[j].acceleration[i] = round_double(accelAvg[i] / (double)accelCounter[i], 5);
				else
					eventData[j].acceleration[i] = PUNK_DBL;
				accelAvg[i] = 0;
			}
			for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
				if (angularRateCounter[i] > 0) {
					if (phid->doZeroGyro && !doneGyroZero)
						eventData[j].angularRate[i] = 0;
					else
						eventData[j].angularRate[i] = round_double(angularRateAvg[i] / (double)angularRateCounter[i], 5);
				} else
					eventData[j].angularRate[i] = PUNK_DBL;
				angularRateAvg[i] = 0;
			}
			for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
				if (magneticFieldCounter[i] > 0)
					eventData[j].magneticField[i] = round_double(magneticFieldAvg[i] / (double)magneticFieldCounter[i], 5);
				else
					eventData[j].magneticField[i] = PUNK_DBL;
				magneticFieldAvg[i] = 0;
			}
			eventData[j].timestamp = tempTime;
		} //for (j = 0;; j++)

		//correct magnetic field data in the event structure
		// But only on devices that don't do this in Firmware!
		switch (phid->phid.deviceInfo.UDD->uid) {
		case PHIDUID_1056:
		case PHIDUID_1056_NEG_GAIN:
			for (j = 0; j < dataPerEvent; j++) {
				for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
					magneticFieldCorr[i] = eventData[j].magneticField[i];
				}
				for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
					if (eventData[j].magneticField[i] != PUNK_DBL) {
						eventData[j].magneticField[i] = getCorrectedField(phid, magneticFieldCorr, i);
					}
				}
			}
			break;
		case PHIDUID_1042:
		case PHIDUID_1044:
		default:
			break;
		}

		//store to local structure
		memset(accelCounter, 0, sizeof(accelCounter));
		memset(angularRateCounter, 0, sizeof(angularRateCounter));
		memset(magneticFieldCounter, 0, sizeof(magneticFieldCounter));
		for (j = 0; j < dataPerEvent; j++) {
			for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
				if (eventData[j].acceleration[i] != PUNK_DBL) {
					accelAvg[i] += eventData[j].acceleration[i];
					accelCounter[i]++;
				}
			}
			for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
				if (eventData[j].angularRate[i] != PUNK_DBL) {
					angularRateAvg[i] += eventData[j].angularRate[i];
					angularRateCounter[i]++;
				}
			}
			for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
				if (eventData[j].magneticField[i] != PUNK_DBL) {
					magneticFieldAvg[i] += eventData[j].magneticField[i];
					magneticFieldCounter[i]++;
				}
			}
		}

		//Set local get data to averages
		for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
			if (accelCounter[i] > 0)
				phid->acceleration[0][i] = round_double(accelAvg[i] / (double)accelCounter[i], 5);
			else
				phid->acceleration[0][i] = PUNK_DBL;
		}
		for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
			if (angularRateCounter[i] > 0) {
				if (phid->doZeroGyro && !doneGyroZero)
					phid->angularRate[0][i] = 0;
				else
					phid->angularRate[0][i] = round_double(angularRateAvg[i] / (double)angularRateCounter[i], 5);
			} else
				phid->angularRate[0][i] = PUNK_DBL;
		}
		for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
			if (magneticFieldCounter[i] > 0)
				phid->magneticField[0][i] = round_double(magneticFieldAvg[i] / (double)magneticFieldCounter[i], 5);
			else
				phid->magneticField[0][i] = PUNK_DBL;
		}

		//send out any events
		for (j = 0; j < dataPerEvent; j++) {
			fireSaturation = PFALSE;
			//If we have opened a spatial - just send the data directly
			if (phid->devChannelCnts.spatial) {
				if ((channel = getChannel(phid, 3)) != NULL) {

					for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
						if (eventData[j].acceleration[i] > phid->accelerationMax || eventData[j].acceleration[i] < phid->accelerationMin)
							fireSaturation |= 0x01;
					}

					for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
						if (eventData[j].angularRate[i] > phid->angularRateMax || eventData[j].angularRate[i] < phid->angularRateMin)
							fireSaturation |= 0x02;
					}

					for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
						if (eventData[j].magneticField[i] == PUNK_DBL)
							continue;
						if (eventData[j].magneticField[i] > phid->magneticFieldMax || eventData[j].magneticField[i] < phid->magneticFieldMin)
							fireSaturation |= 0x04;
					}

					if (fireSaturation == 0) {
						bridgeSendToChannel(channel, BP_SPATIALDATA, "%3G%3G%3G%g", eventData[j].acceleration, eventData[j].angularRate,
							eventData[j].magneticField, eventData[j].timestamp);
					} else {
						if (fireSaturation & 0x01)
							SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Accelerometer Saturation Detected.");
						if (fireSaturation & 0x02)
							SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Gyroscope Saturation Detected.");
						if (fireSaturation & 0x04)
							SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Magnetometer Saturation Detected.");
					}

					PhidgetRelease(&channel);
				}
			}
			if (phid->devChannelCnts.numAccelAxes) {
				int chIndex = 0;
				if ((channel = getChannel(phid, chIndex)) != NULL) {
					fireSaturation = PFALSE;
					fireEvent = PFALSE;
					//If any channel has triggered, or saturated, we send out an event.
					for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++) {
						if (eventData[j].acceleration[i] != PUNK_DBL) {
							if ((fabs(eventData[j].acceleration[i] - phid->accelAxisLastTrigger[i]) >= phid->accelerationChangeTrigger[0] || phid->accelAxisLastTrigger[i] == PUNK_DBL))
								fireEvent = PTRUE;

							if (eventData[j].acceleration[i] > phid->accelerationMax || eventData[j].acceleration[i] < phid->accelerationMin)
								fireSaturation = PTRUE;
						}
					}

					if (fireSaturation)
						SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Accelerometer Saturation Detected.");

					if (fireEvent) {
						bridgeSendToChannel(channel, BP_ACCELERATIONCHANGE, "%3G%g", eventData[j].acceleration, eventData[j].timestamp);
						for (i = 0; i < phid->devChannelCnts.numAccelAxes; i++)
							phid->accelAxisLastTrigger[i] = eventData[j].acceleration[i];
					}

					PhidgetRelease(&channel);
				}
			}
			if (phid->devChannelCnts.numGyroAxes) {
				int chIndex = 1;
				if ((channel = getChannel(phid, chIndex)) != NULL) {
					fireSaturation = PFALSE;
					fireEvent = PFALSE;
					//If any channel has triggered, or saturated, we send out an event.
					for (i = 0; i < phid->devChannelCnts.numGyroAxes; i++) {
						if (eventData[j].angularRate[i] != PUNK_DBL) {
							fireEvent = PTRUE;
							if (eventData[j].angularRate[i] > phid->angularRateMax || eventData[j].angularRate[i] < phid->angularRateMin)
								fireSaturation = PTRUE;
						}
					}

					if (fireSaturation)
						SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Gyroscope Saturation Detected.");

					if (fireEvent)
						bridgeSendToChannel(channel, BP_ANGULARRATEUPDATE, "%3G%g", eventData[j].angularRate, eventData[j].timestamp);

					PhidgetRelease(&channel);
				}
			}

			if (phid->devChannelCnts.numCompassAxes) {
				int chIndex = 2;
				if ((channel = getChannel(phid, chIndex)) != NULL) {
					fireSaturation = PFALSE;
					fireEvent = PFALSE;
					//If any channel has triggered, or saturated, we send out an event.
					for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++) {
						if (eventData[j].magneticField[i] != PUNK_DBL) {
							if ((fabs(eventData[j].magneticField[i] - phid->magAxisLastTrigger[i]) >= phid->magneticFieldChangeTrigger[0] || phid->magAxisLastTrigger[i] == PUNK_DBL))
								fireEvent = PTRUE;
							if (eventData[j].magneticField[i] > phid->magneticFieldMax || eventData[j].magneticField[i] < phid->magneticFieldMin)
								fireSaturation = PTRUE;
						}
					}

					if (fireSaturation)
						SEND_ERROR_EVENT(channel, EEPHIDGET_SATURATION, "Magnetometer Saturation Detected.");

					if (fireEvent) {
						bridgeSendToChannel(channel, BP_FIELDSTRENGTHCHANGE, "%3G%g", eventData[j].magneticField, eventData[j].timestamp);
						for (i = 0; i < phid->devChannelCnts.numCompassAxes; i++)
							phid->magAxisLastTrigger[i] = eventData[j].magneticField[i];
					}

					PhidgetRelease(&channel);
				}
			}
		}

		phid->lastEventTime = phid->latestDataTime;
	}
done:

	//this will signal the zero function to return;
	if (doneGyroZero)
		phid->doZeroGyro = PFALSE;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
PhidgetSpatialDevice_bridgeInput(PhidgetChannelHandle ch, BridgePacket *bp) {
	PhidgetSpatialDeviceHandle phid = (PhidgetSpatialDeviceHandle)ch->parent;

	assert(phid->phid.deviceInfo.class == PHIDCLASS_SPATIAL);

	switch (ch->class) {
	case PHIDCHCLASS_ACCELEROMETER:
		assert(ch->index == 0);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			return (PhidgetSpatialDevice_setDataRate(phid, getBridgePacketUInt32(bp, 0)));
		case BP_SETCHANGETRIGGER:
			phid->accelerationChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_GYROSCOPE:
		assert(ch->index == 0);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			return (PhidgetSpatialDevice_setDataRate(phid, getBridgePacketUInt32(bp, 0)));
		case BP_ZERO:
			return (PhidgetSpatialDevice_zeroGyro(phid));
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_MAGNETOMETER:
		assert(ch->index == 0);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			return (PhidgetSpatialDevice_setDataRate(phid, getBridgePacketUInt32(bp, 0)));
		case BP_SETCHANGETRIGGER:
			phid->magneticFieldChangeTrigger[ch->index] = getBridgePacketDouble(bp, 0);
			return (EPHIDGET_OK);
		case BP_SAVECORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_saveCompassCorrectionParameters(phid));
		case BP_RESETCORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_resetCompassCorrectionParameters(phid));
		case BP_SETCORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_setCompassCorrectionParameters(phid, getBridgePacketDouble(bp, 0), getBridgePacketDouble(bp, 1), getBridgePacketDouble(bp, 2),
				getBridgePacketDouble(bp, 3), getBridgePacketDouble(bp, 4), getBridgePacketDouble(bp, 5), getBridgePacketDouble(bp, 6), getBridgePacketDouble(bp, 7),
				getBridgePacketDouble(bp, 8), getBridgePacketDouble(bp, 9), getBridgePacketDouble(bp, 10), getBridgePacketDouble(bp, 11), getBridgePacketDouble(bp, 12)));
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	case PHIDCHCLASS_SPATIAL:
		assert(ch->index == 0);
		switch (bp->vpkt) {
		case BP_SETDATAINTERVAL:
			return (PhidgetSpatialDevice_setDataRate(phid, getBridgePacketUInt32(bp, 0)));
		case BP_ZERO:
			return (PhidgetSpatialDevice_zeroGyro(phid));
		case BP_RESETCORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_resetCompassCorrectionParameters(phid));
		case BP_SAVECORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_saveCompassCorrectionParameters(phid));
		case BP_SETCORRECTIONPARAMETERS:
			return (PhidgetSpatialDevice_setCompassCorrectionParameters(phid, getBridgePacketDouble(bp, 0), getBridgePacketDouble(bp, 1), getBridgePacketDouble(bp, 2),
				getBridgePacketDouble(bp, 3), getBridgePacketDouble(bp, 4), getBridgePacketDouble(bp, 5), getBridgePacketDouble(bp, 6), getBridgePacketDouble(bp, 7),
				getBridgePacketDouble(bp, 8), getBridgePacketDouble(bp, 9), getBridgePacketDouble(bp, 10), getBridgePacketDouble(bp, 11), getBridgePacketDouble(bp, 12)));
		case BP_OPENRESET:
		case BP_CLOSERESET:
		case BP_ENABLE:
			return (EPHIDGET_OK);
		default:
			MOS_PANIC("Unexpected packet type");
		}

	default:
		MOS_PANIC("Unexpected channel class");
	}
}

static double
getCorrectedField(PhidgetSpatialDeviceHandle phid, double fields[], int axis) {
	switch (axis) {
	case 0:
		return phid->userMagField *
			(phid->userCompassGain[0] * (fields[0] - phid->userCompassOffset[0])
				+ phid->userCompassTransform[0] * (fields[1] - phid->userCompassOffset[1])
				+ phid->userCompassTransform[1] * (fields[2] - phid->userCompassOffset[2]));
	case 1:
		return phid->userMagField *
			(phid->userCompassGain[1] * (fields[1] - phid->userCompassOffset[1])
				+ phid->userCompassTransform[2] * (fields[0] - phid->userCompassOffset[0])
				+ phid->userCompassTransform[3] * (fields[2] - phid->userCompassOffset[2]));
	case 2:
		return phid->userMagField *
			(phid->userCompassGain[2] * (fields[2] - phid->userCompassOffset[2])
				+ phid->userCompassTransform[4] * (fields[0] - phid->userCompassOffset[0])
				+ phid->userCompassTransform[5] * (fields[1] - phid->userCompassOffset[1]));
	default:
		return 0;
	}
}

// TODO: These aren't called from anywhere.. need to eventually expose for manufacturing..
#if 0
// Accel and Gyro tables are the same structure - just different table IDs.
static PhidgetReturnCode
setCalibrationValues_inFirmware(PhidgetSpatialDeviceHandle phid, int tableID, int index,
	double gainPositive[3], double gainNegative[3], double offset[3], double factor1[3], double factor2[3]) {
	uint8_t buffer[SPATIAL_ACCEL_GYRO_CALIB_TABLE_LENGTH] = { 0 };
	int i;
	double offsetMultipliers[3];
	uint32_t header;

	assert(phid);

	if (!deviceSupportsGeneralPacketProtocol((PhidgetDeviceHandle)phid))
		return (EPHIDGET_UNSUPPORTED);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1043:
		if (index == SPATIAL_ANALOG_ACCEL_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = offsetMultipliers[2] = SPATIAL_KXR94_2050_w_AD7689_BITS_PER_G;
			break;
		}
	case PHIDUID_1041:
		if (index == SPATIAL_DIGITAL_ACCEL_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = offsetMultipliers[2] = SPATIAL_MMA8451Q_BITS_PER_G;
			break;
		} else
			return (EPHIDGET_UNSUPPORTED);

	case PHIDUID_1044:
		if (index == SPATIAL_ANALOG_GYRO_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = SPATIAL_LRP410AL_w_AD7689_BITS_PER_DPS;
			offsetMultipliers[2] = SPATIAL_LY330ALH_w_AD7689_BITS_PER_DPS;
			break;
		}
		if (index == SPATIAL_ANALOG_ACCEL_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = offsetMultipliers[2] = SPATIAL_KXR94_2050_w_AD7689_BITS_PER_G;
			break;
		}
	case PHIDUID_1042:
		if (index == SPATIAL_DIGITAL_GYRO_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = offsetMultipliers[2] = SPATIAL_L3GD20_BITS_PER_DPS;
			break;
		}
		if (index == SPATIAL_DIGITAL_ACCEL_CALIB_TABLE_INDEX) {
			offsetMultipliers[0] = offsetMultipliers[1] = offsetMultipliers[2] = SPATIAL_MMA8451Q_BITS_PER_G;
			break;
		} else
			return (EPHIDGET_UNSUPPORTED);
	default:
		return (EPHIDGET_UNSUPPORTED);
	}

	header = (((uint32_t)tableID) << 20) | SPATIAL_ACCEL_GYRO_CALIB_TABLE_LENGTH;

	buffer[3] = header >> 24; //header high byte
	buffer[2] = header >> 16;
	buffer[1] = header >> 8;
	buffer[0] = header >> 0; //header low byte

	for (i = 0; i < 3; i++) {
		int int32Offset = i * 4;
		int temp;

		temp = round(gainPositive[i] * (double)0x10000);
		buffer[int32Offset + 7] = temp >> 24;
		buffer[int32Offset + 6] = temp >> 16;
		buffer[int32Offset + 5] = temp >> 8;
		buffer[int32Offset + 4] = temp >> 0;

		temp = round(gainNegative[i] * (double)0x10000);
		buffer[int32Offset + 19] = temp >> 24;
		buffer[int32Offset + 18] = temp >> 16;
		buffer[int32Offset + 17] = temp >> 8;
		buffer[int32Offset + 16] = temp >> 0;

		temp = round(offset[i] * offsetMultipliers[i]);
		buffer[int32Offset + 31] = temp >> 24;
		buffer[int32Offset + 30] = temp >> 16;
		buffer[int32Offset + 29] = temp >> 8;
		buffer[int32Offset + 28] = temp >> 0;

		temp = round(factor1[i] * (double)0x10000);
		buffer[int32Offset + 43] = temp >> 24;
		buffer[int32Offset + 42] = temp >> 16;
		buffer[int32Offset + 41] = temp >> 8;
		buffer[int32Offset + 40] = temp >> 0;

		temp = round(factor2[i] * (double)0x10000);
		buffer[int32Offset + 55] = temp >> 24;
		buffer[int32Offset + 54] = temp >> 16;
		buffer[int32Offset + 53] = temp >> 8;
		buffer[int32Offset + 52] = temp >> 0;
	}

	return GPP_setDeviceSpecificConfigTable((PhidgetDeviceHandle)phid, buffer, SPATIAL_ACCEL_GYRO_CALIB_TABLE_LENGTH, index);
}
#endif

static PhidgetReturnCode
setCompassCorrectionTable_inFirmware(
	PhidgetSpatialDeviceHandle phid,
	double magField,
	double offset0, double offset1, double offset2,
	double gain0, double gain1, double gain2,
	double T0, double T1, double T2, double T3, double T4, double T5) {
	uint8_t buffer[SPATIAL_COMPASS_CALIB_TABLE_LENGTH] = { 0 };
	int i;
	int gains[3], offsets[3], transforms[6], mag;
	assert(phid);

	if (!deviceSupportsGeneralPacketProtocol((PhidgetDeviceHandle)phid))
		return (EPHIDGET_UNSUPPORTED);

	//compass calibration table Header is: 0x3EA00038
	buffer[3] = 0x3E; //header high byte
	buffer[2] = 0xA0;
	buffer[1] = 0x00;
	buffer[0] = SPATIAL_COMPASS_CALIB_TABLE_LENGTH; //header low byte

	//Mag Field (x0x10000)
	mag = round(magField * (double)0x10000);
	buffer[7] = mag >> 24;
	buffer[6] = mag >> 16;
	buffer[5] = mag >> 8;
	buffer[4] = mag >> 0;

	//Gain (x0x10000)
	gains[0] = round(gain0 * (double)0x10000);
	gains[1] = round(gain1 * (double)0x10000);
	gains[2] = round(gain2 * (double)0x10000);
	for (i = 0; i < 3; i++) {
		buffer[i * 4 + 11] = gains[i] >> 24;
		buffer[i * 4 + 10] = gains[i] >> 16;
		buffer[i * 4 + 9] = gains[i] >> 8;
		buffer[i * 4 + 8] = gains[i] >> 0;
	}

	//Offset
	offsets[0] = round(offset0 * (double)SPATIAL_HMC5883L_BITS_PER_GAUSS);
	offsets[1] = round(offset1 * (double)SPATIAL_HMC5883L_BITS_PER_GAUSS);
	offsets[2] = round(offset2 * (double)SPATIAL_HMC5883L_BITS_PER_GAUSS);
	for (i = 0; i < 3; i++) {
		buffer[i * 4 + 23] = offsets[i] >> 24;
		buffer[i * 4 + 22] = offsets[i] >> 16;
		buffer[i * 4 + 21] = offsets[i] >> 8;
		buffer[i * 4 + 20] = offsets[i] >> 0;
	}

	//Transforms (x0x10000)
	transforms[0] = round(T0 * (double)0x10000);
	transforms[1] = round(T1 * (double)0x10000);
	transforms[2] = round(T2 * (double)0x10000);
	transforms[3] = round(T3 * (double)0x10000);
	transforms[4] = round(T4 * (double)0x10000);
	transforms[5] = round(T5 * (double)0x10000);
	for (i = 0; i < 6; i++) {
		buffer[i * 4 + 35] = transforms[i] >> 24;
		buffer[i * 4 + 34] = transforms[i] >> 16;
		buffer[i * 4 + 33] = transforms[i] >> 8;
		buffer[i * 4 + 32] = transforms[i] >> 0;
	}

	return GPP_setDeviceSpecificConfigTable((PhidgetDeviceHandle)phid, buffer, SPATIAL_COMPASS_CALIB_TABLE_LENGTH, SPATIAL_COMPASS_CALIB_TABLE_INDEX);
}

static void CCONV
PhidgetSpatialDevice_free(PhidgetDeviceHandle *phid) {

	mos_free(*phid, sizeof(struct _PhidgetSpatialDevice));
	*phid = NULL;
}

PhidgetReturnCode
PhidgetSpatialDevice_create(PhidgetSpatialDeviceHandle *phidp) {
	DEVICECREATE_BODY(SpatialDevice, PHIDCLASS_SPATIAL);
	return (EPHIDGET_OK);
}

static PhidgetReturnCode
PhidgetSpatialDevice_setDataRate(PhidgetSpatialDeviceHandle phid, int newVal) {

	//make sure it's a power of 2, or 1
	if (newVal < phid->interruptRate)
		newVal = upper_power_of_two(newVal);
	//make sure it's divisible by interruptRate
	else
		//round to nearest
		newVal = (int)round(newVal / (double)phid->interruptRate) * phid->interruptRate;

	phid->dataInterval[0] = newVal;

	return (EPHIDGET_OK);
}

static PhidgetReturnCode
PhidgetSpatialDevice_zeroGyro(PhidgetSpatialDeviceHandle phid) {
	uint8_t buffer[1];

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1042:
	case PHIDUID_1044:
		buffer[0] = SPATIAL_ZERO_GYRO;
		return (PhidgetDevice_sendpacket((PhidgetDeviceHandle)phid, buffer, 1));
	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:
		if (!phid->doZeroGyro) {
			phid->gyroZeroReadPtr = phid->bufferReadPtr;
			phid->doZeroGyro = PTRUE;
		}
		return (EPHIDGET_OK);
	default:
		return (EPHIDGET_UNEXPECTED);
	}
}

static PhidgetReturnCode
PhidgetSpatialDevice_resetCompassCorrectionParameters(
	PhidgetSpatialDeviceHandle phid) {

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:

		phid->userMagField = 1;

		phid->userCompassOffset[0] = 0;
		phid->userCompassOffset[1] = 0;
		phid->userCompassOffset[2] = 0;

		phid->userCompassGain[0] = 1;
		phid->userCompassGain[1] = 1;
		phid->userCompassGain[2] = 1;

		phid->userCompassTransform[0] = 0;
		phid->userCompassTransform[1] = 0;
		phid->userCompassTransform[2] = 0;
		phid->userCompassTransform[3] = 0;
		phid->userCompassTransform[4] = 0;
		phid->userCompassTransform[5] = 0;
		return (EPHIDGET_OK);

	case PHIDUID_1042:
	case PHIDUID_1044:
		return setCompassCorrectionTable_inFirmware(phid, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0);

	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
PhidgetSpatialDevice_saveCompassCorrectionParameters(
	PhidgetSpatialDeviceHandle phid) {

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1042:
	case PHIDUID_1044:
		return GPP_writeFlash((PhidgetDeviceHandle)phid);

	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
PhidgetSpatialDevice_setCompassCorrectionParameters(
	PhidgetSpatialDeviceHandle phid,
	double magField,
	double offset0, double offset1, double offset2,
	double gain0, double gain1, double gain2,
	double T0, double T1, double T2, double T3, double T4, double T5) {

	//Magnetic Field 0.1-1000
	if (magField < 0.1 || magField > 1000)
		return (EPHIDGET_INVALIDARG);
	//Offsets need to be 0+-5.0
	if (offset0 < -5 || offset0 > 5)
		return (EPHIDGET_INVALIDARG);
	if (offset1 < -5 || offset1 > 5)
		return (EPHIDGET_INVALIDARG);
	if (offset2 < -5 || offset2 > 5)
		return (EPHIDGET_INVALIDARG);
	//Gains need to be 0-15.0
	if (gain0 < 0 || gain0 > 15)
		return (EPHIDGET_INVALIDARG);
	if (gain1 < 0 || gain1 > 15)
		return (EPHIDGET_INVALIDARG);
	if (gain2 < 0 || gain2 > 15)
		return (EPHIDGET_INVALIDARG);
	//T params 0+-5.0
	if (T0 < -5 || T0 > 5)
		return (EPHIDGET_INVALIDARG);
	if (T1 < -5 || T1 > 5)
		return (EPHIDGET_INVALIDARG);
	if (T2 < -5 || T2 > 5)
		return (EPHIDGET_INVALIDARG);
	if (T3 < -5 || T3 > 5)
		return (EPHIDGET_INVALIDARG);
	if (T4 < -5 || T4 > 5)
		return (EPHIDGET_INVALIDARG);
	if (T5 < -5 || T5 > 5)
		return (EPHIDGET_INVALIDARG);

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1056:
	case PHIDUID_1056_NEG_GAIN:

		phid->userMagField = magField;

		phid->userCompassOffset[0] = offset0;
		phid->userCompassOffset[1] = offset1;
		phid->userCompassOffset[2] = offset2;

		phid->userCompassGain[0] = gain0;
		phid->userCompassGain[1] = gain1;
		phid->userCompassGain[2] = gain2;

		phid->userCompassTransform[0] = T0;
		phid->userCompassTransform[1] = T1;
		phid->userCompassTransform[2] = T2;
		phid->userCompassTransform[3] = T3;
		phid->userCompassTransform[4] = T4;
		phid->userCompassTransform[5] = T5;
		return (EPHIDGET_OK);

	case PHIDUID_1042:
	case PHIDUID_1044:
		return setCompassCorrectionTable_inFirmware(phid,
			magField, offset0, offset1, offset2,
			gain0, gain1, gain2, T0, T1, T2, T3, T4, T5);
	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

// TODO: These aren't called from anywhere.. need to eventually expose for manufacturing..
#if 0
PhidgetReturnCode
PhidgetSpatialDevice_setAnalogDigitalMode(PhidgetSpatialDeviceHandle phid, PhidgetSpatialDevice_AnalogDigitalMode newVal) {
	uint8_t buffer[2];

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1043:
	case PHIDUID_1044:
		buffer[0] = SPATIAL_SET_POLLING_TYPE;
		buffer[1] = newVal;
		return (PhidgetDevice_sendpacket((PhidgetDeviceHandle)phid, buffer, 2));
	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
PhidgetSpatialDevice_unZeroGyro(PhidgetSpatialDeviceHandle phid) {
	uint8_t buffer[1];

	switch (phid->phid.deviceInfo.UDD->uid) {
	case PHIDUID_1042:
	case PHIDUID_1044:
		buffer[0] = SPATIAL_UNZERO_GYRO;
		return (PhidgetDevice_sendpacket((PhidgetDeviceHandle)phid, buffer, 1));
	default:
		return (EPHIDGET_UNSUPPORTED);
	}
}

static PhidgetReturnCode
PhidgetSpatialDevice_setDigitalGyroCalibrationValues(PhidgetSpatialDeviceHandle phid,
	double gainPositive[3], double gainNegative[3], double offset[3], double factor1[3], double factor2[3]) {
	return setCalibrationValues_inFirmware(phid, SPATIAL_GyroCalibTable_ID, SPATIAL_DIGITAL_GYRO_CALIB_TABLE_INDEX,
		gainPositive, gainNegative, offset, factor1, factor2);
}

static PhidgetReturnCode
PhidgetSpatialDevice_setAnalogGyroCalibrationValues(PhidgetSpatialDeviceHandle phid,
	double gainPositive[3], double gainNegative[3], double offset[3], double factor1[3], double factor2[3]) {
	return setCalibrationValues_inFirmware(phid, SPATIAL_GyroCalibTable_ID, SPATIAL_ANALOG_GYRO_CALIB_TABLE_INDEX,
		gainPositive, gainNegative, offset, factor1, factor2);
}

static PhidgetReturnCode
PhidgetSpatialDevice_setDigitalAccelCalibrationValues(PhidgetSpatialDeviceHandle phid,
	double gainPositive[3], double gainNegative[3], double offset[3], double factor1[3], double factor2[3]) {
	return setCalibrationValues_inFirmware(phid, SPATIAL_AccelCalibTable_ID, SPATIAL_DIGITAL_ACCEL_CALIB_TABLE_INDEX,
		gainPositive, gainNegative, offset, factor1, factor2);
}

static PhidgetReturnCode
PhidgetSpatialDevice_setAnalogAccelCalibrationValues(PhidgetSpatialDeviceHandle phid,
	double gainPositive[3], double gainNegative[3], double offset[3], double factor1[3], double factor2[3]) {
	return setCalibrationValues_inFirmware(phid, SPATIAL_AccelCalibTable_ID, SPATIAL_ANALOG_ACCEL_CALIB_TABLE_INDEX,
		gainPositive, gainNegative, offset, factor1, factor2);
}
#endif
