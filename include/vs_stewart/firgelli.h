#ifndef _FIRGELLI_H
#define _FIRGELLI_H

#include <libusb-1.0/libusb.h>

class Firgelli{
public:
	Firgelli();
	~Firgelli();
	void SetDebug(int m);
	void Open(int rank);
	int WriteCode(int location, int value);
	void Info();
	int OpenAll();
	int WriteCodeToRank(unsigned int rank, int location, int value);

private:
	int m_interface;
	int interfaces[6];
	libusb_device_handle *m_handle;
	libusb_device_handle* handles[6];
	struct libusb_context *m_ctx;
	static const char *ControlType(int attr);
	static int m_debug;
};
#endif
