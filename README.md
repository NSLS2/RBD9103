# RBD9103

EPICS IOC application for controlling [RBD 9103 PicoAmmeters](https://www.rbdinstruments.com/products/files/downloads/9103//9103-picoammeter-user-guide.pdf).

It is based on the Vendor developed `Actuel` program, and uses the `ASCII` comms protocol as described in the linked manual. The IOC has been tested on Linux systems both with direct access to the device via a USB connection, and through the [DIGI AnyWhereUSB](https://www.digi.com/products/networking/infrastructure-management/usb-connectivity/usb-over-ip/anywhereusb) USB-over-IP solution.

See the [docs](https://github.com/NSLS2/RBD9103/tree/main/docs) directory for an extensive setup guide.

![RBD9103 UI Screenshot](/docs/assets/EXAMPLE_SH_RBD9103.png?raw=True "UI Screenshot")
