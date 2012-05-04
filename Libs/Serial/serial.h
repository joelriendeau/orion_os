/* *****************************************************************************
**   Title   : serial.h
** 
**   Author  : Bruno Sauriol
**   Date    : 18/03/2008
** 
**   Description :
** 
**     Serial communication port driver for Windows header file.
**
**   Copyright Montréal Navigation, 2008. All rights reserved.
**	
** *****************************************************************************
**   Modification history :
** *****************************************************************************
**  Version  |  Mod. date | Author and description
**  
** ****************************************************************************/

#ifndef SERIAL_H     /* prevent circular inclusions */
#define SERIAL_H     /* by using protection macros */

#include "windows.h"

#ifdef __cplusplus
extern "C" {
#endif
    
////////////////////////////////
// Data structures
////////////////////////////////

typedef struct {
    HANDLE hSerial;                 // Serial port handle
    OVERLAPPED osr;                 // Overlapped read I/O handle
    OVERLAPPED osw;                 // Overlapped write I/O handle
    unsigned char comopen;          // Com. port is open flag
    unsigned char *rbuf;            // Receive circular buffer pointer
    long rbufsize;                  // Receive circular buffer size
    long rrptr;                     // Buffer read pointer
    long rwptr;                     // Buffer write pointer
    HANDLE hThread;                 // RX thread handle
	} serialcom;

////////////////////////////////
// Constants
////////////////////////////////
#define COM_RTSDTR      0
#define COM_XONXOFF     1
#define COM_CTSRTS      2

////////////////////////////////
// Function prototypes
////////////////////////////////

long opencom(serialcom* h, unsigned char comport, unsigned long baudrate,
            unsigned char bytesize, unsigned char parity, 
            unsigned char stopbits, char *rxbuf, unsigned long rxbufsize,
            unsigned long flowctl);
long closecom(serialcom* h);
long readcom(serialcom* h, char* buf, unsigned long bufsize);
long writecom(serialcom* h, char* buf, unsigned long bufsize);
long rxbytecnt (serialcom* h);
long rxgetbuf(serialcom* h, char* buf, long bufsize);
long rxpeekbuf(serialcom* h, char* buf, long bufsize);
long rxgetchar (serialcom* h, char *byte);
short rxgetshort (serialcom* h, char byteswap);
long rxgetlong (serialcom* h);
float rxgetfloat (serialcom* h);
double rxgetdouble (serialcom* h);
void rxclrbytes (serialcom* h, unsigned long bytecnt);


#ifdef __cplusplus
}
#endif


#endif          /* end of protection macro */
