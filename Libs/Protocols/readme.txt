A generic comunication protocol implementation. It can support start/stop bytes, Checksum/CRC verification, sequence IDs, etc.

It should be kept highly portable, since it will be used to communicate between the Base and Rover through the RF link, but also between the Rover
and its PDA through either a serial cable, USB, or Bluetooth. Depending on the capabilities already provided by the link, features of the protocol
may be dropped, such as the CRC for Bluetooth and USB.