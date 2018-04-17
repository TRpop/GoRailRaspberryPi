#ifndef _PHIDGETTYPES_GEN_H_
#define _PHIDGETTYPES_GEN_H_

#include "constants.gen.h"


typedef enum {
	ENCODER_IO_MODE_PUSH_PULL = 0x1,
	 	/* No additional pull-up or pull-down resistors will be applied to the input lines. */
	ENCODER_IO_MODE_LINE_DRIVER_2K2 = 0x2,
	 	/* 2.2kΩ pull-down resistors will be applied to the input lines. */
	ENCODER_IO_MODE_LINE_DRIVER_10K = 0x3,
	 	/* 10kΩ pull-down resistors will be applied to the input lines. */
	ENCODER_IO_MODE_OPEN_COLLECTOR_2K2 = 0x4,
	 	/* 2.2kΩ pull-up resistors will be applied to the input lines. */
	ENCODER_IO_MODE_OPEN_COLLECTOR_10K = 0x5,
	 	/* 10kΩ pull-up resistors will be applied to the input lines. */
} Phidget_EncoderIOMode;

typedef enum {
	EPHIDGET_OK = 0x0,	/* Success */
	EPHIDGET_PERM = 0x1,	/* Not Permitted */
	EPHIDGET_NOENT = 0x2,	/* No Such Entity */
	EPHIDGET_TIMEOUT = 0x3,	/* Timed Out */
	EPHIDGET_KEEPALIVE = 0x3a,	/* Keep Alive Failure */
	EPHIDGET_INTERRUPTED = 0x4,	/* Op Interrupted */
	EPHIDGET_IO = 0x5,	/* IO Issue */
	EPHIDGET_NOMEMORY = 0x6,	/* Memory Issue */
	EPHIDGET_ACCESS = 0x7,	/* Access (Permission) Issue */
	EPHIDGET_FAULT = 0x8,	/* Address Issue */
	EPHIDGET_BUSY = 0x9,	/* Resource Busy */
	EPHIDGET_EXIST = 0xa,	/* Object Exists */
	EPHIDGET_NOTDIR = 0xb,	/* Object is not a directory */
	EPHIDGET_ISDIR = 0xc,	/* Object is a directory */
	EPHIDGET_INVALID = 0xd,	/* Invalid */
	EPHIDGET_NFILE = 0xe,	/* Too many open files in system */
	EPHIDGET_MFILE = 0xf,	/* Too many open files */
	EPHIDGET_NOSPC = 0x10,	/* Not enough space */
	EPHIDGET_FBIG = 0x11,	/* File too Big */
	EPHIDGET_ROFS = 0x12,	/* Read Only Filesystem */
	EPHIDGET_RO = 0x13,	/* Read Only Object */
	EPHIDGET_UNSUPPORTED = 0x14,	/* Operation Not Supported */
	EPHIDGET_INVALIDARG = 0x15,	/* Invalid Argument */
	EPHIDGET_AGAIN = 0x16,	/* Try again */
	EPHIDGET_NOTEMPTY = 0x1a,	/* No Empty */
	EPHIDGET_UNEXPECTED = 0x1c,	/* Unexpected Error */
	EPHIDGET_DUPLICATE = 0x1b,	/* Duplicate */
	EPHIDGET_BADPASSWORD = 0x25,	/* Bad Credential */
	EPHIDGET_NETUNAVAIL = 0x2d,	/* Network Unavailable */
	EPHIDGET_CONNREF = 0x23,	/* Connection Refused */
	EPHIDGET_CONNRESET = 0x2e,	/* Connection Reset */
	EPHIDGET_HOSTUNREACH = 0x30,	/* No route to host */
	EPHIDGET_NODEV = 0x28,	/* No Such Device */
	EPHIDGET_WRONGDEVICE = 0x32,	/* Wrong Device */
	EPHIDGET_PIPE = 0x29,	/* Broken Pipe */
	EPHIDGET_RESOLV = 0x2c,	/* Name Resolution Failure */
	EPHIDGET_UNKNOWNVAL = 0x33,	/* Unknown or Invalid Value */
	EPHIDGET_NOTATTACHED = 0x34,	/* Device not Attached */
	EPHIDGET_INVALIDPACKET = 0x35,	/* Invalid or Unexpected Packet */
	EPHIDGET_2BIG = 0x36,	/* Argument List Too Long */
	EPHIDGET_BADVERSION = 0x37,	/* Bad Version */
	EPHIDGET_CLOSED = 0x38,	/* Closed */
	EPHIDGET_NOTCONFIGURED = 0x39,	/* Not Configured */
	EPHIDGET_EOF = 0x1f,	/* End of File */
} PhidgetReturnCode;

typedef enum {
	EEPHIDGET_BADVERSION = 0x1,	/* Client and Server protocol versions don't match. */
	EEPHIDGET_BUSY = 0x2,	/* Phidget is in use. */
	EEPHIDGET_NETWORK = 0x3,	/* Networking communication error. */
	EEPHIDGET_DISPATCH = 0x4,	/* An error occured dispatching a command or event. */
	EEPHIDGET_OK = 0x1000,	/* An error state has cleared. */
	EEPHIDGET_OVERRUN = 0x1002,	/* A sampling overrun happened in firmware. */
	EEPHIDGET_PACKETLOST = 0x1003,	/* One or more packets were lost. */
	EEPHIDGET_WRAP = 0x1004,	/* A variable has wrapped around. */
	EEPHIDGET_OVERTEMP = 0x1005,	/* Over-temperature condition detected. */
	EEPHIDGET_OVERCURRENT = 0x1006,	/* Over-current condition detected. */
	EEPHIDGET_OUTOFRANGE = 0x1007,	/* Out of range condition detected. */
	EEPHIDGET_BADPOWER = 0x1008,	/* Power supply problem detected. */
	EEPHIDGET_SATURATION = 0x1009,	/* Saturation condition detected. */
	EEPHIDGET_OVERVOLTAGE = 0x100b,	/* Over-voltage condition detected. */
	EEPHIDGET_FAILSAFE = 0x100c,	/* Fail-safe condition detected. */
	EEPHIDGET_VOLTAGEERROR = 0x100d,	/* Voltage error detected. */
	EEPHIDGET_ENERGYDUMP = 0x100e,	/* Energy dump condition detected. */
	EEPHIDGET_MOTORSTALL = 0x100f,	/* Motor stall detected. */
} Phidget_ErrorEventCode;

typedef enum {
	PHIDID_NOTHING = 0x0,	/* Unknown device */
	PHIDID_INTERFACEKIT_4_8_8 = 0x1,	/* PhidgetInterfaceKit 4/8/8 */
	PHIDID_1000 = 0x2,	/* PhidgetServo 1-Motor (1000) */
	PHIDID_1001 = 0x3,	/* PhidgetServo 4-Motor (1001) */
	PHIDID_1002 = 0x4,	/* PhidgetAnalog 4-Output (1002) */
	PHIDID_1008 = 0x5,	/* PhidgetAccelerometer 2-Axis (1008) */
	PHIDID_1010_1013_1018_1019 = 0x6,	/* PhidgetInterfaceKit 8/8/8 (1010, 1013, 1018, 1019) */
	PHIDID_1011 = 0x7,	/* PhidgetInterfaceKit 2/2/2 (1011) */
	PHIDID_1012 = 0x8,	/* PhidgetInterfaceKit 0/16/16 (1012) */
	PHIDID_1014 = 0x9,	/* PhidgetInterfaceKit 0/0/4 (1014) */
	PHIDID_1015 = 0xa,	/* PhidgetLinearTouch (1015) */
	PHIDID_1016 = 0xb,	/* PhidgetCircularTouch (1016) */
	PHIDID_1017 = 0xc,	/* PhidgetInterfaceKit 0/0/8 (1017) */
	PHIDID_1023 = 0xd,	/* PhidgetRFID (1023) */
	PHIDID_1024 = 0xe,	/* PhidgetRFID Read-Write (1024) */
	PHIDID_1030 = 0xf,	/* PhidgetLED-64 (1030) */
	PHIDID_1031 = 0x10,	/* PhidgetLED-64 Advanced (1031) */
	PHIDID_1032 = 0x11,	/* PhidgetLED-64 Advanced (1032) */
	PHIDID_1040 = 0x12,	/* PhidgetGPS (1040) */
	PHIDID_1041 = 0x13,	/* PhidgetSpatial 0/0/3 Basic (1041) */
	PHIDID_1042 = 0x14,	/* PhidgetSpatial 3/3/3 Basic (1042) */
	PHIDID_1043 = 0x15,	/* PhidgetSpatial Precision 0/0/3 High Resolution (1043) */
	PHIDID_1044 = 0x16,	/* PhidgetSpatial Precision 3/3/3 High Resolution (1044) */
	PHIDID_1045 = 0x17,	/* PhidgetTemperatureSensor IR (1045) */
	PHIDID_1046 = 0x18,	/* PhidgetBridge 4-Input (1046) */
	PHIDID_1047 = 0x19,	/* PhidgetEncoder HighSpeed 4-Input (1047) */
	PHIDID_1048 = 0x1a,	/* PhidgetTemperatureSensor 4-input (1048) */
	PHIDID_1049 = 0x1b,	/* PhidgetSpatial 0/0/3 (1049) */
	PHIDID_1051 = 0x1c,	/* PhidgetTemperatureSensor 1-Input (1051) */
	PHIDID_1052 = 0x1d,	/* PhidgetEncoder Mechanical (1052) */
	PHIDID_1053 = 0x1e,	/* PhidgetAccelerometer 2-Axis (1053) */
	PHIDID_1054 = 0x1f,	/* PhidgetFrequencyCounter (1054) */
	PHIDID_1055 = 0x20,	/* PhidgetIR (1055) */
	PHIDID_1056 = 0x21,	/* PhidgetSpatial 3/3/3 (1056) */
	PHIDID_1057 = 0x22,	/* PhidgetEncoder HighSpeed (1057) */
	PHIDID_1058 = 0x23,	/* PhidgetPHSensor (1058) */
	PHIDID_1059 = 0x24,	/* PhidgetAccelerometer 3-Axis (1059) */
	PHIDID_1060 = 0x25,	/* PhidgetMotorControl LV (1060) */
	PHIDID_1061 = 0x26,	/* PhidgetAdvancedServo 8-Motor (1061) */
	PHIDID_1062 = 0x27,	/* PhidgetStepper Unipolar 4-Motor (1062) */
	PHIDID_1063 = 0x28,	/* PhidgetStepper Bipolar 1-Motor (1063) */
	PHIDID_1064 = 0x29,	/* PhidgetMotorControl HC (1064) */
	PHIDID_1065 = 0x2a,	/* PhidgetMotorControl 1-Motor (1065) */
	PHIDID_1066 = 0x2b,	/* PhidgetAdvancedServo 1-Motor (1066) */
	PHIDID_1067 = 0x2c,	/* PhidgetStepper Bipolar HC (1067) */
	PHIDID_1202_1203 = 0x2d,	/* PhidgetTextLCD 20x2 with PhidgetInterfaceKit 8/8/8 (1201, 1202,
	  1203) */
	PHIDID_1204 = 0x2e,	/* PhidgetTextLCD Adapter (1204) */
	PHIDID_1215__1218 = 0x2f,	/* PhidgetTextLCD 20x2 (1215/1216/1217/1218) */
	PHIDID_1219__1222 = 0x30,	/* PhidgetTextLCD 20x2 with PhidgetInterfaceKit 0/8/8 (1219, 1220, 1221,
	  1222) */
	PHIDID_ADP1000 = 0x31,	/* pH Adapter */
	PHIDID_ADP1001 = 0x32,	/* SPI Customer Interface */
	PHIDID_DAQ1000 = 0x33,	/* Analog Input Module x8 */
	PHIDID_DAQ1200 = 0x34,	/* Digital Input 4 */
	PHIDID_DAQ1300 = 0x35,	/* Digital Input 4 Isolated */
	PHIDID_DAQ1301 = 0x36,	/* Digital Input 16 */
	PHIDID_DAQ1400 = 0x37,	/* Versatile Input */
	PHIDID_DAQ1500 = 0x38,	/* Bridge */
	PHIDID_DCC1000 = 0x39,	/* DC Motor Controller with PID */
	PHIDID_DST1000 = 0x3a,	/* 200mm Distance Sensor */
	PHIDID_DST1200 = 0x3b,	/* Sonar Distance Sensor */
	PHIDID_ENC1000 = 0x3c,	/* Encoder */
	PHIDID_HIN1000 = 0x3d,	/* Capacitive Touch Sensor */
	PHIDID_HIN1001 = 0x3e,	/* Capacitive Scroll */
	PHIDID_HIN1100 = 0x3f,	/* Joystick */
	PHIDID_HUB0000 = 0x40,	/* Phidget Hub with 6 ports */
	PHIDID_HUB0001 = 0x41,	/* Phidget Mesh Hub with 4 ports */
	PHIDID_HUB0002 = 0x42,	/* Phidget Mesh Dongle */
	PHIDID_HUB0004 = 0x43,	/* Phidget SPI VINT Hub with 6 ports */
	PHIDID_HUB0005 = 0x44,	/* Phidget Lightning Hub with 6 ports */
	PHIDID_HUM1000 = 0x45,	/* Humidity Sensor */
	PHIDID_LCD1100 = 0x46,	/* LCD */
	PHIDID_LED1000 = 0x47,	/* LED Driver 32 */
	PHIDID_LUX1000 = 0x48,	/* Light Sensor */
	PHIDID_MOT1100 = 0x49,	/* Accelerometer 0/0/3 */
	PHIDID_MOT1101 = 0x4a,	/* Spatial 3/3/3 */
	PHIDID_OUT1000 = 0x4b,	/* Analog Output 0-5V */
	PHIDID_OUT1001 = 0x4c,	/* Analog Output (+/-)10V */
	PHIDID_OUT1002 = 0x4d,	/* Analog Output (+/-)10V - 16 bit */
	PHIDID_OUT1100 = 0x4e,	/* Digital Output 4 */
	PHIDID_PRE1000 = 0x4f,	/* Barometer */
	PHIDID_RCC1000 = 0x50,	/* 8-Servo Controller */
	PHIDID_REL1000 = 0x51,	/* Power Relay 4 */
	PHIDID_REL1100 = 0x52,	/* Digital Output 4 Isolated */
	PHIDID_REL1101 = 0x53,	/* Digital Output 16 Isolated */
	PHIDID_SAF1000 = 0x54,	/* Power Supply Protector */
	PHIDID_SND1000 = 0x55,	/* Sound Pressure Level Sensor */
	PHIDID_STC1000 = 0x56,	/* Bipolar Stepper Motor Controller */
	PHIDID_TMP1000 = 0x57,	/* Integrated Temperature Sensor */
	PHIDID_TMP1100 = 0x58,	/* Thermocouple 1 */
	PHIDID_TMP1101 = 0x59,	/* Thermocouple 4 */
	PHIDID_TMP1200 = 0x5a,	/* RTD */
	PHIDID_TMP1300 = 0x5b,	/* Infrared Temperature Sensor */
	PHIDID_VCP1000 = 0x5c,	/* Voltage Sensor High Precision */
	PHIDID_VCP1001 = 0x5d,	/* Voltage Sensor Large */
	PHIDID_VCP1002 = 0x5e,	/* Voltage Sensor Small */
	PHIDID_DIGITALINPUT_PORT = 0x5f,	/* Hub Port in Digital Input mode */
	PHIDID_DIGITALOUTPUT_PORT = 0x60,	/* Hub Port in Digital Output mode */
	PHIDID_VOLTAGEINPUT_PORT = 0x61,	/* Hub Port in Voltage Input mode */
	PHIDID_VOLTAGERATIOINPUT_PORT = 0x62,	/* Hub Port in Voltage Ratio Input mode */
	PHIDID_GENERICUSB = 0x63,	/* Generic USB device */
	PHIDID_GENERICVINT = 0x64,	/* Generic VINT device */
	PHIDID_FIRMWARE_UPGRADE_USB = 0x65,	/* USB device in firmware upgrade mode */
	PHIDID_FIRMWARE_UPGRADE_STM32F0 = 0x66,	/* VINT Device in firmware upgrade mode, STM32F0 Proc. */
	PHIDID_FIRMWARE_UPGRADE_STM8S = 0x67,	/* VINT Device in firmware upgrade mode, STM8S Proc. */
	PHIDID_FIRMWARE_UPGRADE_SPI = 0x68,	/* Phidget SPI device under firmware upgrade */
	PHIDID_VCP1100 = 0x69,	/* 30A Current Sensor */
	PHIDID_VINTSERVO_S2313M = 0x6a,	/* Single Servo S2313M */
	PHIDID_DCC1100 = 0x6c,	/* BLDC Motor Controller */
	PHIDID_HIN1101 = 0x6d,	/* Dial Encoder */
	PHIDID_DCC1001 = 0x6e,	/* Small DC Motor Controller */
	PHIDID_DICTIONARY = 0x6f,	/* Dictionary */
	PHIDID_VINTSERVO_S3317M = 0x70,	/* Single Servo S3317M */
	PHIDID_VINTSERVO_S4309M = 0x71,	/* Single Servo S4309M */
	PHIDID_VINTSERVO_S4309R = 0x72,	/* Single Servo S4309R (Continuous Rotation) */
	PHIDID_STC1001 = 0x73,	/* Bipolar Stepper Motor SmallController */
	PHIDID_USBSWITCH = 0x74,	/* OS Testing Fixture */
	PHIDID_MCSML1 = 0x75,	/* Prototype 4A Small DC Motor controller */
} Phidget_DeviceID;

typedef enum {
	PHIDGET_LOG_CRITICAL = 0x1,	/* Critical */
	PHIDGET_LOG_ERROR = 0x2,	/* Error */
	PHIDGET_LOG_WARNING = 0x3,	/* Warning */
	PHIDGET_LOG_INFO = 0x4,	/* Info */
	PHIDGET_LOG_DEBUG = 0x5,	/* Debug */
	PHIDGET_LOG_VERBOSE = 0x6,	/* Verbose */
} Phidget_LogLevel;

typedef enum {
	PHIDCLASS_NOTHING = 0x0,	/* Any device */
	PHIDCLASS_ACCELEROMETER = 0x1,	/* PhidgetAccelerometer device */
	PHIDCLASS_ADVANCEDSERVO = 0x2,	/* PhidgetAdvancedServo device */
	PHIDCLASS_ANALOG = 0x3,	/* PhidgetAnalog device */
	PHIDCLASS_BRIDGE = 0x4,	/* PhidgetBridge device */
	PHIDCLASS_ENCODER = 0x5,	/* PhidgetEncoder device */
	PHIDCLASS_FREQUENCYCOUNTER = 0x6,	/* PhidgetFrequencyCounter device */
	PHIDCLASS_GPS = 0x7,	/* PhidgetGPS device */
	PHIDCLASS_HUB = 0x8,	/* Phidget VINT Hub device */
	PHIDCLASS_INTERFACEKIT = 0x9,	/* PhidgetInterfaceKit device */
	PHIDCLASS_IR = 0xa,	/* PhidgetIR device */
	PHIDCLASS_LED = 0xb,	/* PhidgetLED device */
	PHIDCLASS_MESHDONGLE = 0xc,	/* Phidget Mesh Dongle */
	PHIDCLASS_MOTORCONTROL = 0xd,	/* PhidgetMotorControl device */
	PHIDCLASS_PHSENSOR = 0xe,	/* PhidgetPHSensor device */
	PHIDCLASS_RFID = 0xf,	/* PhidgetRFID device */
	PHIDCLASS_SERVO = 0x10,	/* PhidgetServo device */
	PHIDCLASS_SPATIAL = 0x11,	/* PhidgetSpatial device */
	PHIDCLASS_STEPPER = 0x12,	/* PhidgetStepper device */
	PHIDCLASS_TEMPERATURESENSOR = 0x13,	/* PhidgetTemperatureSensor device */
	PHIDCLASS_TEXTLCD = 0x14,	/* PhidgetTextLCD device */
	PHIDCLASS_VINT = 0x15,	/* Phidget VINT device */
	PHIDCLASS_GENERIC = 0x16,	/* Generic device */
	PHIDCLASS_FIRMWAREUPGRADE = 0x17,	/* Phidget device in Firmware Upgrade mode */
	PHIDCLASS_DICTIONARY = 0x18,	/* Dictionary device */
} Phidget_DeviceClass;

typedef enum {
	PHIDCHCLASS_NOTHING = 0x0,	/* Any channel */
	PHIDCHCLASS_ACCELEROMETER = 0x1,	/* Accelerometer channel */
	PHIDCHCLASS_CURRENTINPUT = 0x2,	/* Current input channel */
	PHIDCHCLASS_DATAADAPTER = 0x3,	/* Data adapter channel */
	PHIDCHCLASS_DCMOTOR = 0x4,	/* DC motor channel */
	PHIDCHCLASS_DIGITALINPUT = 0x5,	/* Digital input channel */
	PHIDCHCLASS_DIGITALOUTPUT = 0x6,	/* Digital output channel */
	PHIDCHCLASS_DISTANCESENSOR = 0x7,	/* Distance sensor channel */
	PHIDCHCLASS_ENCODER = 0x8,	/* Encoder channel */
	PHIDCHCLASS_FREQUENCYCOUNTER = 0x9,	/* Frequency counter channel */
	PHIDCHCLASS_GPS = 0xa,	/* GPS channel */
	PHIDCHCLASS_LCD = 0xb,	/* LCD channel */
	PHIDCHCLASS_GYROSCOPE = 0xc,	/* Gyroscope channel */
	PHIDCHCLASS_HUB = 0xd,	/* VINT Hub channel */
	PHIDCHCLASS_CAPACITIVETOUCH = 0xe,	/* Capacitive Touch channel */
	PHIDCHCLASS_HUMIDITYSENSOR = 0xf,	/* Humidity sensor channel */
	PHIDCHCLASS_IR = 0x10,	/* IR channel */
	PHIDCHCLASS_LIGHTSENSOR = 0x11,	/* Light sensor channel */
	PHIDCHCLASS_MAGNETOMETER = 0x12,	/* Magnetometer channel */
	PHIDCHCLASS_MESHDONGLE = 0x13,	/* Mesh dongle channel */
	PHIDCHCLASS_PHSENSOR = 0x25,	/* pH sensor channel */
	PHIDCHCLASS_POWERGUARD = 0x14,	/* Power guard channel */
	PHIDCHCLASS_PRESSURESENSOR = 0x15,	/* Pressure sensor channel */
	PHIDCHCLASS_RCSERVO = 0x16,	/* RC Servo channel */
	PHIDCHCLASS_RESISTANCEINPUT = 0x17,	/* Resistance input channel */
	PHIDCHCLASS_RFID = 0x18,	/* RFID channel */
	PHIDCHCLASS_SOUNDSENSOR = 0x19,	/* Sound sensor channel */
	PHIDCHCLASS_SPATIAL = 0x1a,	/* Spatial channel */
	PHIDCHCLASS_STEPPER = 0x1b,	/* Stepper channel */
	PHIDCHCLASS_TEMPERATURESENSOR = 0x1c,	/* Temperature sensor channel */
	PHIDCHCLASS_VOLTAGEINPUT = 0x1d,	/* Voltage input channel */
	PHIDCHCLASS_VOLTAGEOUTPUT = 0x1e,	/* Voltage output channel */
	PHIDCHCLASS_VOLTAGERATIOINPUT = 0x1f,	/* Voltage ratio input channel */
	PHIDCHCLASS_FIRMWAREUPGRADE = 0x20,	/* Firmware upgrade channel */
	PHIDCHCLASS_GENERIC = 0x21,	/* Generic channel */
	PHIDCHCLASS_MOTORPOSITIONCONTROLLER = 0x22,	/* Motor position control channel. */
	PHIDCHCLASS_BLDCMOTOR = 0x23,	/* BLDC motor channel */
	PHIDCHCLASS_DICTIONARY = 0x24,	/* Dictionary */
} Phidget_ChannelClass;

typedef enum {
	PHIDCHSUBCLASS_NONE = 0x1,	/* No subclass */
	PHIDCHSUBCLASS_DIGITALOUTPUT_DUTY_CYCLE = 0x10,	/* Digital output duty cycle */
	PHIDCHSUBCLASS_DIGITALOUTPUT_LED_DRIVER = 0x11,	/* Digital output LED driver */
	PHIDCHSUBCLASS_TEMPERATURESENSOR_RTD = 0x20,	/* Temperature sensor RTD */
	PHIDCHSUBCLASS_TEMPERATURESENSOR_THERMOCOUPLE = 0x21,	/* Temperature sensor thermocouple */
	PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT = 0x30,	/* Voltage sensor port */
	PHIDCHSUBCLASS_VOLTAGERATIOINPUT_SENSOR_PORT = 0x40,	/* Voltage ratio sensor port */
	PHIDCHSUBCLASS_VOLTAGERATIOINPUT_BRIDGE = 0x41,	/* Voltage ratio bridge input */
	PHIDCHSUBCLASS_LCD_GRAPHIC = 0x50,	/* Graphic LCD */
	PHIDCHSUBCLASS_LCD_TEXT = 0x51,	/* Text LCD */
	PHIDCHSUBCLASS_ENCODER_MODE_SETTABLE = 0x60,	/* Encoder IO mode settable */
	PHIDCHSUBCLASS_RCSERVO_EMBEDDED = 0x70,	/* RC Servo Embedded */
} Phidget_ChannelSubclass;

typedef enum {
	MESHMODE_ROUTER = 0x1,	/* Router mode */
	MESHMODE_SLEEPYENDDEVICE = 0x2,	/* Sleepy end device mode */
} Phidget_MeshMode;

typedef enum {
	POWER_SUPPLY_OFF = 0x1,	/* Switch the sensor power supply off */
	POWER_SUPPLY_12V = 0x2,	/* The sensor is provided with 12 volts */
	POWER_SUPPLY_24V = 0x3,	/* The sensor is provided with 24 volts */
} Phidget_PowerSupply;

typedef enum {
	RTD_WIRE_SETUP_2WIRE = 0x1,
	 	/* Configures the device to make resistance calculations based on a 2-wire RTD setup. */
	RTD_WIRE_SETUP_3WIRE = 0x2,
	 	/* Configures the device to make resistance calculations based on a 3-wire RTD setup. */
	RTD_WIRE_SETUP_4WIRE = 0x3,
	 	/* Configures the device to make resistance calculations based on a 4-wire RTD setup. */
} Phidget_RTDWireSetup;

typedef enum {
	INPUT_MODE_NPN = 0x1,	/* For interfacing NPN digital sensors */
	INPUT_MODE_PNP = 0x2,	/* For interfacing PNP digital sensors */
} Phidget_InputMode;

typedef enum {
	FAN_MODE_OFF = 0x1,	/* Turns the fan off. */
	FAN_MODE_ON = 0x2,	/* Turns the fan on. */
	FAN_MODE_AUTO = 0x3,	/* The fan will be automatically controlled based on temperature. */
} Phidget_FanMode;

typedef enum {
	PHIDUNIT_NONE = 0x0,	/* Unitless */
	PHIDUNIT_BOOLEAN = 0x1,	/* Boolean */
	PHIDUNIT_PERCENT = 0x2,	/* Percent */
	PHIDUNIT_DECIBEL = 0x3,	/* Decibel */
	PHIDUNIT_MILLIMETER = 0x4,	/* Millimeter */
	PHIDUNIT_CENTIMETER = 0x5,	/* Centimeter */
	PHIDUNIT_METER = 0x6,	/* Meter */
	PHIDUNIT_GRAM = 0x7,	/* Gram */
	PHIDUNIT_KILOGRAM = 0x8,	/* Kilogram */
	PHIDUNIT_MILLIAMPERE = 0x9,	/* Milliampere */
	PHIDUNIT_AMPERE = 0xa,	/* Ampere */
	PHIDUNIT_KILOPASCAL = 0xb,	/* Kilopascal */
	PHIDUNIT_VOLT = 0xc,	/* Volt */
	PHIDUNIT_DEGREE_CELCIUS = 0xd,	/* Degree Celcius */
	PHIDUNIT_LUX = 0xe,	/* Lux */
	PHIDUNIT_GAUSS = 0xf,	/* Gauss */
	PHIDUNIT_PH = 0x10,	/* pH */
	PHIDUNIT_WATT = 0x11,	/* Watt */
} Phidget_Unit;

typedef enum {
	LED_FORWARD_VOLTAGE_1_7V = 0x1,	/* 1.7 V */
	LED_FORWARD_VOLTAGE_2_75V = 0x2,	/* 2.75 V */
	LED_FORWARD_VOLTAGE_3_2V = 0x3,	/* 3.2 V */
	LED_FORWARD_VOLTAGE_3_9V = 0x4,	/* 3.9 V */
	LED_FORWARD_VOLTAGE_4_0V = 0x5,	/* 4.0 V */
	LED_FORWARD_VOLTAGE_4_8V = 0x6,	/* 4.8 V */
	LED_FORWARD_VOLTAGE_5_0V = 0x7,	/* 5.0 V */
	LED_FORWARD_VOLTAGE_5_6V = 0x8,	/* 5.6 V */
} PhidgetDigitalOutput_LEDForwardVoltage;

typedef enum {
	FILTER_TYPE_ZERO_CROSSING = 0x1,
	 	/* Frequency is calculated from the number of times the signal transitions from a negative voltage to a positive voltage and back again. */
	FILTER_TYPE_LOGIC_LEVEL = 0x2,
	 	/* Frequency is calculated from the number of times the signal transitions from a logic false to a logic true and back again. */
} PhidgetFrequencyCounter_FilterType;

typedef struct {
	int16_t tm_ms;
	int16_t tm_sec;
	int16_t tm_min;
	int16_t tm_hour;
} PhidgetGPS_Time, *PhidgetGPS_TimeHandle;

typedef struct {
	int16_t tm_mday;
	int16_t tm_mon;
	int16_t tm_year;
} PhidgetGPS_Date, *PhidgetGPS_DateHandle;

typedef struct {
	double latitude;
	double longitude;
	int16_t fixQuality;
	int16_t numSatellites;
	double horizontalDilution;
	double altitude;
	double heightOfGeoid;
} PhidgetGPS_GPGGA, *PhidgetGPS_GPGGAHandle;

typedef struct {
	char mode;
	int16_t fixType;
	int16_t satUsed[12];
	double posnDilution;
	double horizDilution;
	double vertDilution;
} PhidgetGPS_GPGSA, *PhidgetGPS_GPGSAHandle;

typedef struct {
	char status;
	double latitude;
	double longitude;
	double speedKnots;
	double heading;
	double magneticVariation;
	char mode;
} PhidgetGPS_GPRMC, *PhidgetGPS_GPRMCHandle;

typedef struct {
	double trueHeading;
	double magneticHeading;
	double speedKnots;
	double speed;
	char mode;
} PhidgetGPS_GPVTG, *PhidgetGPS_GPVTGHandle;

typedef struct {
	PhidgetGPS_GPGGA GGA;
	PhidgetGPS_GPGSA GSA;
	PhidgetGPS_GPRMC RMC;
	PhidgetGPS_GPVTG VTG;
} PhidgetGPS_NMEAData, *PhidgetGPS_NMEADataHandle;

typedef enum {
	PORT_MODE_VINT_PORT = 0x0,	/* Communicate with a smart VINT device */
	PORT_MODE_DIGITAL_INPUT = 0x1,	/* 5V Logic-level digital input */
	PORT_MODE_DIGITAL_OUTPUT = 0x2,	/* 3.3V digital output */
	PORT_MODE_VOLTAGE_INPUT = 0x3,	/* 0-5V voltage input for non-ratiometric sensors  */
	PORT_MODE_VOLTAGE_RATIO_INPUT = 0x4,	/* 0-5V voltage input for ratiometric sensors */
} PhidgetHub_PortMode;

typedef enum {
	IR_ENCODING_UNKNOWN = 0x1,	/* Unknown - the default value */
	IR_ENCODING_SPACE = 0x2,	/* Space encoding, or Pulse Distance Modulation */
	IR_ENCODING_PULSE = 0x3,	/* Pulse encoding, or Pulse Width Modulation */
	IR_ENCODING_BIPHASE = 0x4,	/* Bi-Phase, or Manchester encoding */
	IR_ENCODING_RC5 = 0x5,	/* RC5 - a type of Bi-Phase encoding */
	IR_ENCODING_RC6 = 0x6,	/* RC6 - a type of Bi-Phase encoding */
} PhidgetIR_Encoding;

typedef enum {
	IR_LENGTH_UNKNOWN = 0x1,	/* Unknown - the default value */
	IR_LENGTH_CONSTANT = 0x2,	/* Constant - the bitstream and gap length is constant */
	IR_LENGTH_VARIABLE = 0x3,	/* Variable - the bitstream has a variable length with a constant gap */
} PhidgetIR_Length;

typedef struct {
	uint32_t bitCount;
	PhidgetIR_Encoding encoding;
	PhidgetIR_Length length;
	uint32_t gap;
	uint32_t trail;
	uint32_t header[2];
	uint32_t one[2];
	uint32_t zero[2];
	uint32_t repeat[26];
	uint32_t minRepeat;
	double dutyCycle;
	uint32_t carrierFrequency;
	char toggleMask[33];
} PhidgetIR_CodeInfo, *PhidgetIR_CodeInfoHandle;

typedef enum {
	FONT_User1 = 0x1,	/* User-defined font #1 */
	FONT_User2 = 0x2,	/* User-defined font #2 */
	FONT_6x10 = 0x3,	/* 6px by 10px font */
	FONT_5x8 = 0x4,	/* 5px by 8px font */
	FONT_6x12 = 0x5,	/* 6px by 12px font */
} PhidgetLCD_Font;

typedef enum {
	SCREEN_SIZE_NONE = 0x1,	/* Screen size unknown */
	SCREEN_SIZE_1x8 = 0x2,	/* One row, eight column text screen */
	SCREEN_SIZE_2x8 = 0x3,	/* Two row, eight column text screen */
	SCREEN_SIZE_1x16 = 0x4,	/* One row, 16 column text screen */
	SCREEN_SIZE_2x16 = 0x5,	/* Two row, 16 column text screen */
	SCREEN_SIZE_4x16 = 0x6,	/* Four row, 16 column text screen */
	SCREEN_SIZE_2x20 = 0x7,	/* Two row, 20 column text screen */
	SCREEN_SIZE_4x20 = 0x8,	/* Four row, 20 column text screen. */
	SCREEN_SIZE_2x24 = 0x9,	/* Two row, 24 column text screen */
	SCREEN_SIZE_1x40 = 0xa,	/* One row, 40 column text screen */
	SCREEN_SIZE_2x40 = 0xb,	/* Two row, 40 column text screen */
	SCREEN_SIZE_4x40 = 0xc,	/* Four row, 40 column text screen */
	SCREEN_SIZE_64x128 = 0xd,	/* 64px by 128px graphic screen */
} PhidgetLCD_ScreenSize;

typedef enum {
	PIXEL_STATE_OFF = 0x0,	/* Pixel off state */
	PIXEL_STATE_ON = 0x1,	/* Pixel on state */
	PIXEL_STATE_INVERT = 0x2,	/* Invert the pixel state */
} PhidgetLCD_PixelState;

typedef struct {
	Phidget_Unit unit;
	const char * name;
	const char * symbol;
} Phidget_UnitInfo, *Phidget_UnitInfoHandle;

typedef enum {
	PHIDGETSERVER_NONE = 0x0,	/* Unknown or unspecified server type */
	PHIDGETSERVER_DEVICELISTENER = 0x1,	/* Phidget22 Server listener */
	PHIDGETSERVER_DEVICE = 0x2,	/* Phidget22 Server connection */
	PHIDGETSERVER_DEVICEREMOTE = 0x3,
	 	/* Phidget22 Server<br/>Server discovery with this server type allows discovery of servers hosting Phidget devices. Enabling server discovery with this server type allows automated connection to these servers, and the Phidgets connected to them. Enabling server discovery with this server type will also enable ServerAdded and ServerRemoved events for this server type. */
	PHIDGETSERVER_WWWLISTENER = 0x4,	/* Phidget22 Web Server */
	PHIDGETSERVER_WWW = 0x5,	/* Phidget22 Web Server connection */
	PHIDGETSERVER_WWWREMOTE = 0x6,
	 	/* Phidget22 Web server<br/>Server discovery with this server type detects the presence of Phidget web servers used to communicate with in-browser JavaScript. Enabling server discovery with this server type will enable ServerAdded and ServerRemoved events for this server type. */
	PHIDGETSERVER_SBC = 0x7,
	 	/* Phidget SBC<br/>Server discovery with this server type detects the presence of Phidget SBCs on the network. Enabling server discovery with this server type will enable ServerAdded and ServerRemoved events for this server type. */
} PhidgetServerType;

typedef enum {
	RCSERVO_VOLTAGE_5V = 0x1,	/* Run all servos on 5V DC */
	RCSERVO_VOLTAGE_6V = 0x2,	/* Run all servos on 6V DC */
	RCSERVO_VOLTAGE_7_4V = 0x3,	/* Run all servos on 7.4V DC */
} PhidgetRCServo_Voltage;

typedef enum {
	PROTOCOL_EM4100 = 0x1,	/* EM4100 */
	PROTOCOL_ISO11785_FDX_B = 0x2,	/* ISO11785 FDX B */
	PROTOCOL_PHIDGETS = 0x3,	/* PhidgetTAG */
} PhidgetRFID_Protocol;

typedef enum {
	SPL_RANGE_102dB = 0x1,	/* Range 102dB */
	SPL_RANGE_82dB = 0x2,	/* Range 82dB */
} PhidgetSoundSensor_SPLRange;

typedef enum {
	CONTROL_MODE_STEP = 0x0,	/* Control the motor by setting a target position. */
	CONTROL_MODE_RUN = 0x1,
	 	/* Control the motor by selecting a target velocity (sign indicates direction). The motor will rotate continously in the chosen direction. */
} PhidgetStepper_ControlMode;

typedef enum {
	RTD_TYPE_PT100_3850 = 0x1,	/* Configures the RTD type as a PT100 with a 3850ppm curve. */
	RTD_TYPE_PT1000_3850 = 0x2,	/* Configures the RTD type as a PT1000 with a 3850ppm curve. */
	RTD_TYPE_PT100_3920 = 0x3,	/* Configures the RTD type as a PT100 with a 3920ppm curve. */
	RTD_TYPE_PT1000_3920 = 0x4,	/* Configures the RTD type as a PT1000 with a 3920ppm curve. */
} PhidgetTemperatureSensor_RTDType;

typedef enum {
	THERMOCOUPLE_TYPE_J = 0x1,	/* Configures the thermocouple input as a J-Type thermocouple. */
	THERMOCOUPLE_TYPE_K = 0x2,	/* Configures the thermocouple input as a K-Type thermocouple. */
	THERMOCOUPLE_TYPE_E = 0x3,	/* Configures the thermocouple input as a E-Type thermocouple. */
	THERMOCOUPLE_TYPE_T = 0x4,	/* Configures the thermocouple input as a T-Type thermocouple. */
} PhidgetTemperatureSensor_ThermocoupleType;

typedef enum {
	VOLTAGE_RANGE_10mV = 0x1,	/* Range ±10mV DC */
	VOLTAGE_RANGE_40mV = 0x2,	/* Range ±40mV DC */
	VOLTAGE_RANGE_200mV = 0x3,	/* Range ±200mV DC */
	VOLTAGE_RANGE_312_5mV = 0x4,	/* Range ±312.5mV DC */
	VOLTAGE_RANGE_400mV = 0x5,	/* Range ±400mV DC */
	VOLTAGE_RANGE_1000mV = 0x6,	/* Range ±1000mV DC */
	VOLTAGE_RANGE_2V = 0x7,	/* Range ±2V DC */
	VOLTAGE_RANGE_5V = 0x8,	/* Range ±5V DC */
	VOLTAGE_RANGE_15V = 0x9,	/* Range ±15V DC */
	VOLTAGE_RANGE_40V = 0xa,	/* Range ±40V DC */
	VOLTAGE_RANGE_AUTO = 0xb,	/* Auto-range mode changes based on the present voltage measurements. */
} PhidgetVoltageInput_VoltageRange;

typedef enum {
	SENSOR_TYPE_VOLTAGE = 0x0,
	 	/* Default. Configures the channel to be a generic voltage sensor. Unit is volts. */
	SENSOR_TYPE_1114 = 0x2b84,	/* 1114 - Temperature Sensor */
	SENSOR_TYPE_1117 = 0x2ba2,	/* 1117 - Voltage Sensor */
	SENSOR_TYPE_1123 = 0x2bde,	/* 1123 - Precision Voltage Sensor */
	SENSOR_TYPE_1127 = 0x2c06,	/* 1127 - Precision Light Sensor */
	SENSOR_TYPE_1130_PH = 0x2c25,	/* 1130 - pH Adapter */
	SENSOR_TYPE_1130_ORP = 0x2c26,	/* 1130 - ORP Adapter */
	SENSOR_TYPE_1132 = 0x2c38,	/* 1132 - 4-20mA Adapter */
	SENSOR_TYPE_1133 = 0x2c42,	/* 1133 - Sound Sensor */
	SENSOR_TYPE_1135 = 0x2c56,	/* 1135 - Precision Voltage Sensor */
	SENSOR_TYPE_1142 = 0x2c9c,	/* 1142 - Light Sensor 1000 lux */
	SENSOR_TYPE_1143 = 0x2ca6,	/* 1143 - Light Sensor 70000 lux */
	SENSOR_TYPE_3500 = 0x88b8,	/* 3500 - AC Current Sensor 10Amp */
	SENSOR_TYPE_3501 = 0x88c2,	/* 3501 - AC Current Sensor 25Amp */
	SENSOR_TYPE_3502 = 0x88cc,	/* 3502 - AC Current Sensor 50Amp */
	SENSOR_TYPE_3503 = 0x88d6,	/* 3503 - AC Current Sensor 100Amp */
	SENSOR_TYPE_3507 = 0x88fe,	/* 3507 - AC Voltage Sensor 0-250V (50Hz) */
	SENSOR_TYPE_3508 = 0x8908,	/* 3508 - AC Voltage Sensor 0-250V (60Hz) */
	SENSOR_TYPE_3509 = 0x8912,	/* 3509 - DC Voltage Sensor 0-200V */
	SENSOR_TYPE_3510 = 0x891c,	/* 3510 - DC Voltage Sensor 0-75V */
	SENSOR_TYPE_3511 = 0x8926,	/* 3511 - DC Current Sensor 0-10mA */
	SENSOR_TYPE_3512 = 0x8930,	/* 3512 - DC Current Sensor 0-100mA */
	SENSOR_TYPE_3513 = 0x893a,	/* 3513 - DC Current Sensor 0-1A */
	SENSOR_TYPE_3514 = 0x8944,	/* 3514 - AC Active Power Sensor 0-250V*0-30A (50Hz) */
	SENSOR_TYPE_3515 = 0x894e,	/* 3515 - AC Active Power Sensor 0-250V*0-30A (60Hz) */
	SENSOR_TYPE_3516 = 0x8958,	/* 3516 - AC Active Power Sensor 0-250V*0-5A (50Hz) */
	SENSOR_TYPE_3517 = 0x8962,	/* 3517 - AC Active Power Sensor 0-250V*0-5A (60Hz) */
	SENSOR_TYPE_3518 = 0x896c,	/* 3518 - AC Active Power Sensor 0-110V*0-5A (60Hz) */
	SENSOR_TYPE_3519 = 0x8976,	/* 3519 - AC Active Power Sensor 0-110V*0-15A (60Hz) */
	SENSOR_TYPE_3584 = 0x8c00,	/* 3584 - 0-50A DC Current Transducer */
	SENSOR_TYPE_3585 = 0x8c0a,	/* 3585 - 0-100A DC Current Transducer */
	SENSOR_TYPE_3586 = 0x8c14,	/* 3586 - 0-250A DC Current Transducer */
	SENSOR_TYPE_3587 = 0x8c1e,	/* 3587 - +-50A DC Current Transducer */
	SENSOR_TYPE_3588 = 0x8c28,	/* 3588 - +-100A DC Current Transducer */
	SENSOR_TYPE_3589 = 0x8c32,	/* 3589 - +-250A DC Current Transducer */
} PhidgetVoltageInput_SensorType;

typedef enum {
	VOLTAGE_OUTPUT_RANGE_10V = 0x1,	/* ±10V DC */
	VOLTAGE_OUTPUT_RANGE_5V = 0x2,	/* 0-5V DC */
} PhidgetVoltageOutput_VoltageOutputRange;

typedef enum {
	BRIDGE_GAIN_1 = 0x1,	/* 1x Amplificaion */
	BRIDGE_GAIN_2 = 0x2,	/* 2x Amplification */
	BRIDGE_GAIN_4 = 0x3,	/* 4x Amplification */
	BRIDGE_GAIN_8 = 0x4,	/* 8x Amplification */
	BRIDGE_GAIN_16 = 0x5,	/* 16x Amplification */
	BRIDGE_GAIN_32 = 0x6,	/* 32x Amplification */
	BRIDGE_GAIN_64 = 0x7,	/* 64x Amplification */
	BRIDGE_GAIN_128 = 0x8,	/* 128x Amplification */
} PhidgetVoltageRatioInput_BridgeGain;

typedef enum {
	SENSOR_TYPE_VOLTAGERATIO = 0x0,
	 	/* Default. Configures the channel to be a generic ratiometric sensor. Unit is volts/volt. */
	SENSOR_TYPE_1101_SHARP_2D120X = 0x2b03,	/* 1101 - IR Distance Adapter,
	  with Sharp Distance Sensor 2D120X (4-30cm) */
	SENSOR_TYPE_1101_SHARP_2Y0A21 = 0x2b04,	/* 1101 - IR Distance Adapter,
	  with Sharp Distance Sensor 2Y0A21 (10-80cm) */
	SENSOR_TYPE_1101_SHARP_2Y0A02 = 0x2b05,	/* 1101 - IR Distance Adapter,
	  with Sharp Distance Sensor 2Y0A02 (20-150cm) */
	SENSOR_TYPE_1102 = 0x2b0c,	/* 1102 - IR Reflective Sensor 5mm */
	SENSOR_TYPE_1103 = 0x2b16,	/* 1103 - IR Reflective Sensor 10cm */
	SENSOR_TYPE_1104 = 0x2b20,	/* 1104 - Vibration Sensor */
	SENSOR_TYPE_1105 = 0x2b2a,	/* 1105 - Light Sensor */
	SENSOR_TYPE_1106 = 0x2b34,	/* 1106 - Force Sensor */
	SENSOR_TYPE_1107 = 0x2b3e,	/* 1107 - Humidity Sensor */
	SENSOR_TYPE_1108 = 0x2b48,	/* 1108 - Magnetic Sensor */
	SENSOR_TYPE_1109 = 0x2b52,	/* 1109 - Rotation Sensor */
	SENSOR_TYPE_1110 = 0x2b5c,	/* 1110 - Touch Sensor */
	SENSOR_TYPE_1111 = 0x2b66,	/* 1111 - Motion Sensor */
	SENSOR_TYPE_1112 = 0x2b70,	/* 1112 - Slider 60 */
	SENSOR_TYPE_1113 = 0x2b7a,	/* 1113 - Mini Joy Stick Sensor */
	SENSOR_TYPE_1115 = 0x2b8e,	/* 1115 - Pressure Sensor */
	SENSOR_TYPE_1116 = 0x2b98,	/* 1116 - Multi-turn Rotation Sensor */
	SENSOR_TYPE_1118_AC = 0x2bad,	/* 1118 - 50Amp Current Sensor AC */
	SENSOR_TYPE_1118_DC = 0x2bae,	/* 1118 - 50Amp Current Sensor DC */
	SENSOR_TYPE_1119_AC = 0x2bb7,	/* 1119 - 20Amp Current Sensor AC */
	SENSOR_TYPE_1119_DC = 0x2bb8,	/* 1119 - 20Amp Current Sensor DC */
	SENSOR_TYPE_1120 = 0x2bc0,	/* 1120 - FlexiForce Adapter */
	SENSOR_TYPE_1121 = 0x2bca,	/* 1121 - Voltage Divider */
	SENSOR_TYPE_1122_AC = 0x2bd5,	/* 1122 - 30 Amp Current Sensor AC */
	SENSOR_TYPE_1122_DC = 0x2bd6,	/* 1122 - 30 Amp Current Sensor DC */
	SENSOR_TYPE_1124 = 0x2be8,	/* 1124 - Precision Temperature Sensor */
	SENSOR_TYPE_1125_HUMIDITY = 0x2bf3,	/* 1125 - Humidity Sensor */
	SENSOR_TYPE_1125_TEMPERATURE = 0x2bf4,	/* 1125 - Temperature Sensor */
	SENSOR_TYPE_1126 = 0x2bfc,	/* 1126 - Differential Air Pressure Sensor +- 25kPa */
	SENSOR_TYPE_1128 = 0x2c10,	/* 1128 - MaxBotix EZ-1 Sonar Sensor */
	SENSOR_TYPE_1129 = 0x2c1a,	/* 1129 - Touch Sensor */
	SENSOR_TYPE_1131 = 0x2c2e,	/* 1131 - Thin Force Sensor */
	SENSOR_TYPE_1134 = 0x2c4c,	/* 1134 - Switchable Voltage Divider */
	SENSOR_TYPE_1136 = 0x2c60,	/* 1136 - Differential Air Pressure Sensor +-2 kPa */
	SENSOR_TYPE_1137 = 0x2c6a,	/* 1137 - Differential Air Pressure Sensor +-7 kPa */
	SENSOR_TYPE_1138 = 0x2c74,	/* 1138 - Differential Air Pressure Sensor 50 kPa */
	SENSOR_TYPE_1139 = 0x2c7e,	/* 1139 - Differential Air Pressure Sensor 100 kPa */
	SENSOR_TYPE_1140 = 0x2c88,	/* 1140 - Absolute Air Pressure Sensor 20-400 kPa */
	SENSOR_TYPE_1141 = 0x2c92,	/* 1141 - Absolute Air Pressure Sensor 15-115 kPa */
	SENSOR_TYPE_1146 = 0x2cc4,	/* 1146 - IR Reflective Sensor 1-4mm */
	SENSOR_TYPE_3120 = 0x79e0,	/* 3120 - Compression Load Cell (0-4.5 kg) */
	SENSOR_TYPE_3121 = 0x79ea,	/* 3121 - Compression Load Cell (0-11.3 kg) */
	SENSOR_TYPE_3122 = 0x79f4,	/* 3122 - Compression Load Cell (0-22.7 kg) */
	SENSOR_TYPE_3123 = 0x79fe,	/* 3123 - Compression Load Cell (0-45.3 kg) */
	SENSOR_TYPE_3130 = 0x7a44,	/* 3130 - Relative Humidity Sensor */
	SENSOR_TYPE_3520 = 0x8980,	/* 3520 - Sharp Distance Sensor (4-30cm) */
	SENSOR_TYPE_3521 = 0x898a,	/* 3521 - Sharp Distance Sensor (10-80cm) */
	SENSOR_TYPE_3522 = 0x8994,	/* 3522 - Sharp Distance Sensor (20-150cm) */
} PhidgetVoltageRatioInput_SensorType;

#endif /* _PHIDGETTYPES_GEN_H_ */
