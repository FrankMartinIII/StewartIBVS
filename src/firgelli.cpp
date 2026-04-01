#include "firgelli.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

int Firgelli::m_debug;


Firgelli::Firgelli()
{
	m_ctx=NULL;
	m_handle=NULL;
	m_interface=0;
	for(int i = 0 ; i < 6 ; i++)
	{
		handles[i] = NULL;
		interfaces[i] = 0;
	}
	libusb_init(&m_ctx);
}

Firgelli::~Firgelli()
{
	int rval=libusb_release_interface(m_handle,m_interface);
	assert(rval==0);
	
	if(m_handle)
		libusb_close(m_handle);
	libusb_exit(m_ctx);
}

void Firgelli::SetDebug(int m)
{
	libusb_set_debug(m_ctx,m);
	m_debug=m;
}

void Firgelli::Open(int rank)
{
	int rval;
	uint16_t vid=0x04d8;		// Microtech
	uint16_t pid=0xfc5f;		// ??
	
#ifdef SIMPLE
	m_handle=libusb_open_device_with_vid_pid(m_ctx,vid,pid);
	if(m_handle==NULL){
		fprintf(stderr,"firgelli::open device vid=0x%x pid=0x%x not found\n",
			vid,pid);
		exit(1);
	}
#else
	libusb_device **devs;
	int count=libusb_get_device_list(m_ctx,&devs);
	if(m_debug)
		printf("number of usb devices=%d\n",count);
	if(count == 0){
		fprintf(stderr,"no device found\n");
		exit(1);
	}
	if(count < 0){
		fprintf(stderr,"device get error\n");
		exit(1);
	}
	int nfound=0;
	libusb_device *found=NULL;
	for(int i=0; i<count; i++){
		libusb_device *dev=devs[i];
		libusb_device_descriptor ddesc;
		int rval=libusb_get_device_descriptor(dev,&ddesc);
		if(rval<0){
			fprintf(stderr,"device descriptor get error\n");
			exit(1);
		}
		if((ddesc.idVendor == vid) && (ddesc.idProduct == pid)){
			nfound++;
			if (nfound==rank)
				found=dev;
		}
	}
	if(m_debug)
		printf("number of lacs=%d\n",nfound);
	if(found==NULL){
		fprintf(stderr,"device rank=%d not found\n",rank);
		exit(1);
	}
	rval=libusb_open(found,&m_handle);
	assert(rval==0);	
#endif
	rval=libusb_claim_interface(m_handle,m_interface);
	printf("m_interface: %d\n", m_interface);
	assert(rval==0);
}

/*
int Firgelli::OpenAll()
{
	int rval;
	uint16_t vid=0x04d8;		// Microtech
	uint16_t pid=0xfc5f;		// ??
	libusb_device **devs;
	
	int count=libusb_get_device_list(m_ctx,&devs);
	if(m_debug)
	{
		printf("number of usb devices=%d\n",count);
	}
	if(count == 0)
	{
		fprintf(stderr,"no device found\n");
		exit(1);
	}
	if(count < 0)
	{
		fprintf(stderr,"device get error\n");
		exit(1);
	}
	
	int nfound=0;
	libusb_device *found=NULL;
	
	for(int i=0; i<count; i++)
	{
		libusb_device *dev=devs[i]; //get the device
		libusb_device_descriptor ddesc;
		int rval=libusb_get_device_descriptor(dev,&ddesc);
		if(rval<0)
		{
			fprintf(stderr,"device descriptor get error\n");
			exit(1);
		}
		if((ddesc.idVendor == vid) && (ddesc.idProduct == pid))
		{
			found=dev;
			if(nfound < 6)
			{
				rval=libusb_open(found, &(handles[nfound]));
				if(rval == LIBUSB_ERROR_NO_MEM)
				{
					printf("memory allocation error opening LAC\n");
				}
				else if(rval == LIBUSB_ERROR_ACCESS)
				{
					printf("Insufficient permissions to open LAC USB\n"); 
				}
				else if(rval == LIBUSB_ERROR_NO_DEVICE)
				{
					printf("USB disconnected\n");
				}
				else if(rval == 0)
				{
					printf("LAC %d opened successfully\n", nfound);
				}	
				else
				{
					printf("Weird error %d\n", rval);
				}
				
				rval=libusb_claim_interface(handles[nfound],interfaces[nfound]);
				if(rval == LIBUSB_ERROR_NOT_FOUND)
				{
					printf("Interface does not exist\n");
				}
				else if(rval == LIBUSB_ERROR_BUSY)
				{
					printf("Interface busy\n"); 
				}
				else if(rval == LIBUSB_ERROR_NO_DEVICE)
				{
					printf("USB disconnected\n");
				}
				else if(rval == 0)
				{
					printf("Interface %d opened successfully\n", interfaces[nfound]);
				}	
				else
				{
					printf("Weird error %d\n", rval);
				}
			}
			nfound++;
		}
	}
	libusb_free_device_list(devs,0);
	printf("number of lacs=%d\n",nfound);
	if(nfound != 6)
	{
		fprintf(stderr, "Incorrect number of LACs found. Hexapod should have 6 LACs");
		//exit(1);
	}
    return nfound;
}
*/

int Firgelli::OpenAll()
{
	int rval;
	uint16_t vid=0x04d8;		// Microtech
	uint16_t pid=0xfc5f;		// ??
	libusb_device **devs;
	
	int count=libusb_get_device_list(m_ctx,&devs);
	if(m_debug)
	{
		printf("number of usb devices=%d\n",count);
	}
	if(count == 0)
	{
		fprintf(stderr,"no device found\n");
		exit(1);
	}
	if(count < 0)
	{
		fprintf(stderr,"device get error\n");
		exit(1);
	}

	//Port mapping. Maps each LAC to the correct port on the USB hub. 
	//The 0th element of each row is the length of the path.
	int expectedPaths[6][8] = {
		{3,6,1,1,0,0,0,0},
		{3,6,1,2,0,0,0,0},
		{3,6,1,3,0,0,0,0},
		{3,6,1,4,0,0,0,0},
		{2,6,2,0,0,0,0,0},
		{2,6,3,0,0,0,0,0}

	};
	
	int nfound=0;
	libusb_device *found=NULL;
	
	for(int i=0; i<count; i++)
	{
		libusb_device *dev=devs[i]; //get the device
		libusb_device_descriptor ddesc;
		int rval=libusb_get_device_descriptor(dev,&ddesc);
		if(rval<0)
		{
			fprintf(stderr,"device descriptor get error\n");
			exit(1);
		}
		if((ddesc.idVendor == vid) && (ddesc.idProduct == pid))
		{
			uint8_t portPath[7];
			found=dev;
			
			//Get the port path of the device
			int pathLen = libusb_get_port_numbers(found, portPath, sizeof(portPath)) ; 
			int assignedRank = -1;
			for (int rank = 0 ; rank < 6 ; rank++) //Loop thru all potential expectedPaths
			{
				if(expectedPaths[rank][0] == pathLen) //Make sure path length returned is correct and matches
				{
					//If the length matches, check if the rest of the path matches
					bool match = true;
					for(int p = 0 ; p < pathLen ; p++)
					{
						if(expectedPaths[rank][p+1] != portPath[p])
						{
							match = false;
							break;
						}

					}
					if(match) //If the path matches we can assign
					{
						assignedRank = rank;
						break;
					}
				}
			}
			
			if(assignedRank != -1)
			{
				//Rank successfully matched to array entry
				if(handles[assignedRank] == NULL)
				{
					rval = libusb_open(found, &(handles[assignedRank]));
					if(rval == LIBUSB_ERROR_NO_MEM)
					{
						printf("memory allocation error opening LAC\n");
					}
					else if(rval == LIBUSB_ERROR_ACCESS)
					{
						printf("Insufficient permissions to open LAC USB\n"); 
					}
					else if(rval == LIBUSB_ERROR_NO_DEVICE)
					{
						printf("USB disconnected\n");
					}
					else if(rval == 0)
					{
						printf("LAC mapped to rank %d successfully (Path: ", assignedRank);
						printf("LAC %d opened successfully\n", assignedRank);
					}	
					else
					{
						printf("Weird error %d\n", rval);
					}
					
					rval=libusb_claim_interface(handles[assignedRank],interfaces[assignedRank]);
					if(rval == LIBUSB_ERROR_NOT_FOUND)
					{
						printf("Interface does not exist\n");
					}
					else if(rval == LIBUSB_ERROR_BUSY)
					{
						printf("Interface busy\n"); 
					}
					else if(rval == LIBUSB_ERROR_NO_DEVICE)
					{
						printf("USB disconnected\n");
					}
					else if(rval == 0)
					{
						printf("Interface %d opened successfully\n", interfaces[assignedRank]);
						nfound++;
					}	
					else
					{
						printf("Weird error %d\n", rval);
					}
				}

				else
				{
					printf("Warning: Multiple devices matching path for rank %d. Check mapping array.\n", assignedRank);
				}

			}
			else //LAC found with path that was not known prior
			{
				printf("Found unmapped LAC. Path: ");
				for(int p = 0; p < pathLen; p++) printf("%d ", portPath[p]);
				printf("\n");
			}

		}
	}
	libusb_free_device_list(devs,0);
	printf("number of lacs=%d\n",nfound);
	if(nfound != 6)
	{
		fprintf(stderr, "Incorrect number of LACs found. Hexapod should have 6 LACs");
		//exit(1);
	}
    return nfound;
}


#define NBUF	80

void Firgelli::Info()
{
//	printf("version=%d\n",libusb_get_version());

	int rval=libusb_kernel_driver_active(m_handle,m_interface);
	printf("kernel driver active=%d\n",rval);

//	char buf[NBUF];
//	rval=libusb_get_string_descriptor_ascii(m_handle,

	libusb_device *device=libusb_get_device(m_handle);
	int busno=libusb_get_bus_number(device);	
	int address=libusb_get_device_address(device);	
	printf("busno=%d address=%d\n",busno,address);

	libusb_device_descriptor ddesc;
	rval=libusb_get_device_descriptor(device,&ddesc);
	if(rval < 0 ){
		fprintf(stderr,"firgelli: bad descriptor\n");
		exit(1);
	}
	printf("number of configurations=%d\n",ddesc.bNumConfigurations);
	printf("serial number=%d\n",ddesc.iSerialNumber);
	printf("productid=0x%x\n",ddesc.idProduct);

	int c;
	rval=libusb_get_configuration(m_handle,&c);
	assert(rval==0);
	printf("config=%d\n",c);

	libusb_config_descriptor *cdesc;
	rval=libusb_get_config_descriptor(device,0,&cdesc);
	assert(rval==0);
	printf("number of interfaces=%d\n",cdesc->bNumInterfaces);
	struct libusb_interface interface=cdesc->interface[0];
	const struct libusb_interface_descriptor *idex=interface.altsetting;

	int nend=idex->bNumEndpoints;
	printf("nend=%d\n",nend);
	const struct libusb_endpoint_descriptor *ends=idex->endpoint;
	for(int i=0; i<nend; i++){
		int eaddr=ends[i].bEndpointAddress;
		int attr=ends[i].bmAttributes;
		printf("addr=%d attr=0x%x %s\n",eaddr,attr,ControlType(attr));
	}

	libusb_free_config_descriptor(cdesc);
}

int Firgelli::WriteCode(int location, int value)
{
	int transferred=0;
	unsigned int timeout=1000;
	unsigned char buf[3];

	buf[0]=location;
	buf[1]=value%256;	// Low
	buf[2]=value/256;	// High

	unsigned char endpoint=1 | LIBUSB_ENDPOINT_OUT; 

	int rval=libusb_bulk_transfer(
		m_handle,
		endpoint,	
		buf,
		3,
		&transferred,
		timeout);
	if(m_debug){
		printf("location=0x%x value=0x%x endpoint=%d transferred=%d\n",
		location,value,endpoint,transferred);
		printf("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x\n",
			buf[0], buf[1],buf[2]);
	}
	if(rval ){
		fprintf(stderr,"firgelli: bad write rval=%d\n",rval);
		exit(1);
	}

	endpoint=1 | LIBUSB_ENDPOINT_IN;
	rval=libusb_bulk_transfer(
		m_handle,
		endpoint,	
		buf,
		3,
		&transferred,
		timeout);
	if(m_debug)
		printf("location=0x%x endpoint=%d transferred=%d\n",
			location,endpoint,transferred);

	if(rval ){
		fprintf(stderr,"firgelli: bad read rval=%d \n",
			rval);
		exit(1);
	}
	if(m_debug)
		printf("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x\n",
		buf[0], buf[1],buf[2]);
	
	return buf[1]+256*buf[2];

}

int Firgelli::WriteCodeToRank(unsigned int rank, int location, int value)
{
	printf("WriteCodeToRank\n");
        if(rank < 0 || rank > 6)
        {
        	printf("Illegal rank value\n");
        	return 1;
        }
	int transferred=0;
	unsigned int timeout=1000;
	unsigned char buf[3];

	buf[0]=location;
	buf[1]=value%256;	// Low
	buf[2]=value/256;	// High

	unsigned char endpoint=1 | LIBUSB_ENDPOINT_OUT; 

	int rval=libusb_bulk_transfer(
		handles[rank],
		endpoint,	
		buf,
		3,
		&transferred,
		timeout);
	if(m_debug){
		printf("LAC#=%d location=0x%x value=0x%x endpoint=%d transferred=%d\n",
		rank,location,value,endpoint,transferred);
		printf("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x\n",
			buf[0], buf[1],buf[2]);
	}
	if(rval ){
		fprintf(stderr,"firgelli: bad write rval=%d\n",rval);
		exit(1);
	}

	endpoint=1 | LIBUSB_ENDPOINT_IN;
	rval=libusb_bulk_transfer(
		handles[rank],
		endpoint,	
		buf,
		3,
		&transferred,
		timeout);
	if(m_debug)
		printf("LAC#=%d location=0x%x endpoint=%d transferred=%d\n",
			rank,location,endpoint,transferred);

	if(rval ){
		fprintf(stderr,"firgelli: bad read rval=%d \n",
			rval);
		exit(1);
	}
	if(m_debug)
		printf("buf[0]=0x%x buf[1]=0x%x buf[2]=0x%x\n",
		buf[0], buf[1],buf[2]);
	
	return buf[1]+256*buf[2];

}

const char *Firgelli::ControlType(int attr)
{
	attr=attr&0x03;		// mask bits 1 and 2
	switch(attr){
		case LIBUSB_TRANSFER_TYPE_CONTROL:
			return "control";
		case LIBUSB_TRANSFER_TYPE_BULK:
			return "bulk";
	}
	return "unknown";
}
