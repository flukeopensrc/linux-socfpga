#ifndef __NRPZMODULE_H_
#define __NRPZMODULE_H_

#include "linux/ioctl.h"

#define NRPZ_IOC_MAGIC              'N'

#define NRPZ_GETSENSORINFO          _IOR(NRPZ_IOC_MAGIC, 0x01, struct nrpz_sensor_info *)
#define NRPZ_START                  _IO(NRPZ_IOC_MAGIC, 0x02)
#define NRPZ_WRITE_DONE             _IOW(NRPZ_IOC_MAGIC, 0x03, unsigned int)
#define NRPZ_VENDOR_CONTROL_MSG     _IOW(NRPZ_IOC_MAGIC, 0x06, struct nrpz_control_req *)
#define NRPZ_VENDOR_CONTROL_MSG_OUT _IOW(NRPZ_IOC_MAGIC, 0x06, struct nrpz_control_req *)
#define NRPZ_VENDOR_CONTROL_MSG_IN  _IOW(NRPZ_IOC_MAGIC, 0x07, struct nrpz_control_req *)
#define NRPZ_GETDEVICEREADY         _IO(NRPZ_IOC_MAGIC, 0x08)
#define NRPZ_LEGACYOPEN             _IO(NRPZ_IOC_MAGIC, 0x09)
#define NRPZ_LEGACYCLOSE            _IO(NRPZ_IOC_MAGIC, 0x0a)

struct nrpz_sensor_info {
    unsigned char bDescriptorType;
    unsigned short bcdUSB;
    unsigned char bDeviceClass;
    unsigned char bDeviceSubClass;
    unsigned char bDeviceProtocol;
    unsigned char bMaxPacketSize0;
    unsigned short vendorId;
    unsigned short productId;
    unsigned short bcdDevice;
    unsigned char iManufacturer;
    unsigned char iProduct;
    unsigned char iSerialNumber;
    unsigned char bNumConfigurations;
    char protocol[128];
    char manufacturer[128];
    char productName[128];
    char serialNumber[128];
};

/*
 * struct for NRPZ_VENDOR_CONTROL
 */
struct nrpz_control_req {
    unsigned char request;
    unsigned char type;
    unsigned short value;
    unsigned short index;
    unsigned char *data;
    unsigned short size;
};

#endif
