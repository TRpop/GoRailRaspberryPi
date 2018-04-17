#include "errorstrings.gen.h"

const char * CCONV
Phidget_strerror(PhidgetReturnCode code) {

	switch (code) {
	case EPHIDGET_OK:
		return ("Success");
	case EPHIDGET_PERM:
		return ("Not Permitted");
	case EPHIDGET_NOENT:
		return ("No Such Entity");
	case EPHIDGET_TIMEOUT:
		return ("Timed Out");
	case EPHIDGET_KEEPALIVE:
		return ("Keep Alive Failure");
	case EPHIDGET_INTERRUPTED:
		return ("Op Interrupted");
	case EPHIDGET_IO:
		return ("IO Issue");
	case EPHIDGET_NOMEMORY:
		return ("Memory Issue");
	case EPHIDGET_ACCESS:
		return ("Access (Permission) Issue");
	case EPHIDGET_FAULT:
		return ("Address Issue");
	case EPHIDGET_BUSY:
		return ("Resource Busy");
	case EPHIDGET_EXIST:
		return ("Object Exists");
	case EPHIDGET_NOTDIR:
		return ("Object is not a directory");
	case EPHIDGET_ISDIR:
		return ("Object is a directory");
	case EPHIDGET_INVALID:
		return ("Invalid");
	case EPHIDGET_NFILE:
		return ("Too many open files in system");
	case EPHIDGET_MFILE:
		return ("Too many open files");
	case EPHIDGET_NOSPC:
		return ("Not enough space");
	case EPHIDGET_FBIG:
		return ("File too Big");
	case EPHIDGET_ROFS:
		return ("Read Only Filesystem");
	case EPHIDGET_RO:
		return ("Read Only Object");
	case EPHIDGET_UNSUPPORTED:
		return ("Operation Not Supported");
	case EPHIDGET_INVALIDARG:
		return ("Invalid Argument");
	case EPHIDGET_AGAIN:
		return ("Try again");
	case EPHIDGET_NOTEMPTY:
		return ("No Empty");
	case EPHIDGET_UNEXPECTED:
		return ("Unexpected Error");
	case EPHIDGET_DUPLICATE:
		return ("Duplicate");
	case EPHIDGET_BADPASSWORD:
		return ("Bad Credential");
	case EPHIDGET_NETUNAVAIL:
		return ("Network Unavailable");
	case EPHIDGET_CONNREF:
		return ("Connection Refused");
	case EPHIDGET_CONNRESET:
		return ("Connection Reset");
	case EPHIDGET_HOSTUNREACH:
		return ("No route to host");
	case EPHIDGET_NODEV:
		return ("No Such Device");
	case EPHIDGET_WRONGDEVICE:
		return ("Wrong Device");
	case EPHIDGET_PIPE:
		return ("Broken Pipe");
	case EPHIDGET_RESOLV:
		return ("Name Resolution Failure");
	case EPHIDGET_UNKNOWNVAL:
		return ("Unknown or Invalid Value");
	case EPHIDGET_NOTATTACHED:
		return ("Device not Attached");
	case EPHIDGET_INVALIDPACKET:
		return ("Invalid or Unexpected Packet");
	case EPHIDGET_2BIG:
		return ("Argument List Too Long");
	case EPHIDGET_BADVERSION:
		return ("Bad Version");
	case EPHIDGET_CLOSED:
		return ("Closed");
	case EPHIDGET_NOTCONFIGURED:
		return ("Not Configured");
	case EPHIDGET_EOF:
		return ("End of File");
	default:
		return ("Invalid return code");
	}
}
