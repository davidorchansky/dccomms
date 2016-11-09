# dccomms

libdccomms.so project

## Description
This library contains some useful utils to develop communication protocols.
This project contains the source code of libdccomms.so and some Arduino sketches as examples of how to implement a bridge between a computer and a radio transmitter for sending the Data-Link Frames generated by dccomms (*dccomms::DataLinkFrame*)


## Examples:

### Arduino as a Data-Link Frame bridge

The Arduino bridge examples offer a *half-duplex* TX -> RX communicaction link between two RF based communication nodes. Each arduino acts as a simple bridge of the data link frames generated by *dccomms*.

In the Arduino code, the following constant:
```c++
#define FCS_SIZE 4
```
must have a value according to the CRC type used by each *DataLinkFrame*: 4 for *crc32*, 2 for *crc16*, and 0 for *nofcs*.

Upload each Arduino sketch (TX y RX) according to the transducer model. The Arduino sketches are in the folder *src/Arduino*:

* [*BiM2A 433 MHz*](http://www.radiometrix.com/content/bim2a).
* [*BiM3B 868 MHz*](http://www.radiometrix.com/content/bim3b) (El mismo programa que para BiM2A).
* [*FS1000A 315 MHz*](http://www.ananiahelectronics.com/fs100a.gif).
* [*nR24l01 2.4 GHz*](http://elecfreaks.com/store/download/datasheet/rf/rf24l01_PA_LAN/nRF24L01P.PDF)

## Compilation

### Dependencies

This project has a temporal dependency with the *libcrypto.so* for MD5 calculation, needed for integrity checking of large blocks of data (used by *dccomms::BlockDataTransmitter*):

		sudo apt-get install libssl-dev 

Also, for logging, it depends on the submodule [*cpplogging*](https://github.com/dcentelles/cpplogging)
