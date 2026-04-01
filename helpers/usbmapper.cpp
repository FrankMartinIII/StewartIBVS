#include <stdio.h>
#include <stdlib.h>
#include <libusb-1.0/libusb.h>

int main() {
    libusb_context *ctx = NULL;
    libusb_device **devs;
    int rval = libusb_init(&ctx);
    
    if (rval < 0) {
        fprintf(stderr, "Failed to initialize libusb\n");
        return 1;
    }

    int count = libusb_get_device_list(ctx, &devs);
    if (count < 0) {
        fprintf(stderr, "Failed to get device list\n");
        return 1;
    }

    printf("Scanning for Actuonix LAC boards (VID: 0x04d8, PID: 0xfc5f)...\n\n");
    int lac_count = 0;

    for (int i = 0; i < count; i++) {
        libusb_device *dev = devs[i];
        libusb_device_descriptor ddesc;
        libusb_get_device_descriptor(dev, &ddesc);

        // Match the Microtech/Actuonix VID and PID
        if (ddesc.idVendor == 0x04d8 && ddesc.idProduct == 0xfc5f) {
            lac_count++;
            uint8_t port_path[7];
            int path_len = libusb_get_port_numbers(dev, port_path, sizeof(port_path));
            
            int bus_num = libusb_get_bus_number(dev);
            int dev_addr = libusb_get_device_address(dev);

            printf("LAC #%d found at Bus %d, Address %d\n", lac_count, bus_num, dev_addr);
            printf("--> Physical Port Path: ");
            for (int j = 0; j < path_len; j++) {
                printf("%d ", port_path[j]);
            }
            printf("\n\n");
        }
    }

    if (lac_count == 0) {
        printf("No LAC boards found. Check your USB connection.\n");
    } else {
        printf("Total LAC boards found: %d\n", lac_count);
    }

    libusb_free_device_list(devs, 1);
    libusb_exit(ctx);
    return 0;
}