/* *****************************************************************************
**   Title   : serial.c
** 
**   Author  : Bruno Sauriol
**   Date    : 20/03/2008
** 
**   Description :
** 
**     Serial communication port driver for Windows.
**
**   Copyright Montréal Navigation, 2008. All rights reserved.
**	
** *****************************************************************************
**   Modification history :
** *****************************************************************************
**  Version  |  Mod. date | Author and description
**  
** ****************************************************************************/

#include "windows.h"
#include "stdio.h"
#include "serial.h"

/////////////////////////
// PRIVATE FUNCTIONS
/////////////////////////

/*********************************************************************
* Function: serialrx
**********************************************************************
* In:  Serial port handle
* Out: Data from serial port
*
* Description:
*   This function runs in background as a thread. It fills up a
*   circular buffer with data received from a serial port.
*
*********************************************************************/
unsigned long WINAPI SerialRX (void * Param) {

    serialcom *h = Param;
    long rbytes = 0;
    long cbytes;
    long rsize;
    long i;
    char buf[1024];

    while (h->comopen) {
        // Fetches as much data as possible
        rsize = h->rbufsize - rxbytecnt(h) - 1;
        if (rsize > 1) {
            if (rsize > 1024) rsize = 1024;
            rbytes = readcom(h, buf,  rsize);

            if (rbytes > 0) {
                // Copies data to circular buffer
                cbytes = 0;
                for (i = h->rwptr; i < h->rbufsize; i++) {
                    h->rbuf[i] = buf[cbytes++];
                    if (cbytes == rbytes) break;
                }
                if (cbytes < rbytes) {
                    for (i = 0; i < h->rrptr; i++) {
                        h->rbuf[i] = buf[cbytes++];
                        if (cbytes >= rbytes) break;
                    }
                }
                i++;
                if (i >= h->rbufsize) i -= h->rbufsize;
                h->rwptr = i;
            } else if (rbytes == -1) {
                // Error encountered: exit thread
                break;
            } else {
                // No data available: wait for 1 msec
                //Sleep(1);
            }
        } else {
            // Circular buffer is full: wait for 1 msec
            Sleep(1);
        }
    }

    if (rbytes == -1)
        return 1;
    else
        return 0;
}


/////////////////////////
// PUBLIC FUNCTIONS
/////////////////////////

/*********************************************************************
* Function: opencom
**********************************************************************
* In:  Serial port configuration
* Out: Serial port handle
*
* Description:
*   Opens a serial communication port
*
*********************************************************************/
long opencom(serialcom* h, unsigned char comport, unsigned long baudrate,
            unsigned char bytesize, unsigned char parity, 
            unsigned char stopbits, char *rxbuf, unsigned long rxbufsize,
            unsigned long flowctl) {

    unsigned long ThreadId;
    WCHAR com[10] = L"\\\\.\\COM0";
    DCB dcb;
    COMMTIMEOUTS timeout;

    memset(h,0,sizeof(*h));     // Resets COM handle

    if (comport > 99) return 0; // Bad COM port

    if (comport > 9) {          // Generates COM port string
        com[7] = comport / 10;
        com[8] = comport % 10;
        com[9] = 0;
        com[7] += '0';
        com[8] += '0';
    } else {
        com[7] = comport + '0';
    }

    // Opens COM port
    h->hSerial = CreateFile(com,
                            GENERIC_READ | GENERIC_WRITE,
                            0,
                            NULL,
                            OPEN_EXISTING,
                            FILE_FLAG_OVERLAPPED, //0,
                            NULL);

    if (h->hSerial == INVALID_HANDLE_VALUE) return 1; // Error encountered


    // Create I/O overlapped handlew
    h->osr.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    h->osw.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    // Initialize the rest of the OVERLAPPED structures to zero.
    h->osr.Internal = 0;
    h->osr.InternalHigh = 0;
    h->osr.Offset = 0;
    h->osr.OffsetHigh = 0;
    h->osw.Internal = 0;
    h->osw.InternalHigh = 0;
    h->osw.Offset = 0;
    h->osw.OffsetHigh = 0;


    // Init. COM port
    memset(&dcb,0,sizeof(dcb));
    dcb.DCBlength = 28;
    dcb.BaudRate = baudrate;
    dcb.ByteSize = bytesize;
    dcb.Parity = parity;
    dcb.StopBits = stopbits - 1;
	dcb.fBinary  = 1;

    switch (flowctl) {
    case COM_RTSDTR:
        dcb.fDtrControl	= 1;
	    dcb.fRtsControl	= 1;
        break;
    case COM_XONXOFF:
        dcb.XonLim = 2048;
        dcb.XoffLim = 512;
        dcb.XonChar = 17;
        dcb.XoffChar = 19;
    }

    if (!SetCommState(h->hSerial,&dcb)) {
        closecom(h);
        return 1; // Error encountered
    }

    timeout.ReadIntervalTimeout = 1;
    timeout.ReadTotalTimeoutMultiplier = 1;
    timeout.ReadTotalTimeoutConstant = 1;
    timeout.WriteTotalTimeoutMultiplier = 1;
    timeout.WriteTotalTimeoutConstant = 5000;

    if (!SetCommTimeouts(h->hSerial,&timeout)) {
        closecom(h);
        return 1; // Error encountered
    }
    
    h->rbuf = rxbuf;            // Initializes circular buffer
    h->rbufsize = rxbufsize;    //
    h->rrptr = 0;               //
    h->rwptr = 0;               //

    h->comopen = 1;             // COM port is open flag

    // Creates RX thread
    h->hThread = CreateThread(NULL, 0, SerialRX, h, 0, &ThreadId); 

	// Setup thread priority
	SetPriorityClass(h->hThread, REALTIME_PRIORITY_CLASS);
	SetThreadPriority(h->hThread, THREAD_PRIORITY_TIME_CRITICAL);

    // Operation completed succesfully
    return 0;
}


/*********************************************************************
* Function: closecom
**********************************************************************
* In:  Serial port handle
* Out: Error flag
*
* Description:
*   Closes a serial communication port
*
*********************************************************************/
long closecom(serialcom* h) {

    h->comopen = 0; // COM port is close

    if (!CloseHandle(h->hSerial))
        return 1;
    else
        return 0;
}


/*********************************************************************
* Function: readcom
**********************************************************************
* In:  Serial port handle, read buffer pointer and size
* Out: Data ready flag
*
* Description:
*   Reads bytes from a serial communication port.
*
*********************************************************************/
long readcom(serialcom* h, char* buf, unsigned long bufsize) {

    unsigned long dwBytesRead;
    long status;

    ReadFile(h->hSerial,buf,bufsize, &dwBytesRead,  &(h->osr));
    status = GetLastError();

    if (status == 0) {
        return dwBytesRead; // Read succeeded
    } else if (status == ERROR_IO_PENDING) {
        if(GetOverlappedResult(h->hSerial, &(h->osr), &dwBytesRead, TRUE))
            return dwBytesRead; // Read succeeded
        else
            return -1;          // Error encountered
    } else {
        return -1;      // Error encountered
    }
}


/*********************************************************************
* Function: writecom
**********************************************************************
* In:  Serial port handle, write buffer pointer and size
* Out: Error flag
*
* Description:
*   Writes bytes to a serial communication port.
*
*********************************************************************/
long writecom(serialcom* h, char* buf, unsigned long bufsize) {

    unsigned long dwBytesWriten;
    long status;

    WriteFile(h->hSerial,buf,bufsize, &dwBytesWriten, &(h->osw));
    status = GetLastError();

    if (status == ERROR_IO_PENDING) {
        if(GetOverlappedResult(h->hSerial, &(h->osw), &dwBytesWriten, TRUE))
            return 0;  // Write succeeded
        else
            return 1;  // Error encountered
    } else {
        return 1;      // Error encountered
    }
}


/*********************************************************************
* Function: rxbytecnt
**********************************************************************
* In:  Serial port handle
* Out: RX buffer size
*
* Description:
*   Returns the RX buffer size in bytes
*
*********************************************************************/
long rxbytecnt (serialcom* h) {

    unsigned long wptr = h->rwptr;
    unsigned long rptr = h->rrptr;

    if (wptr < rptr)
        return h->rbufsize - (rptr - wptr);
    else
        return wptr - rptr;
}


/*********************************************************************
* Function: rxgetbuf
**********************************************************************
* In:  Serial port handle, read buffer pointer and size
* Out: Data ready flag
*
* Description:
*   Copies byte buffer from RX buffer.
*
*********************************************************************/
long rxgetbuf(serialcom* h, char* buf, long bufsize) {

    long i;
    if (rxbytecnt(h) < bufsize)
        return 1; // Not enough data in RX buffer
    if (buf != 0)
    {
        // Copy bytes
        for (i = 0; i < bufsize; i++) {
            *buf++ = h->rbuf[h->rrptr++];
            if (h->rrptr >= h->rbufsize) h->rrptr = 0;
        }
    }
    else
    {
        // Flush bytes
        h->rrptr += bufsize;
        if (h->rrptr >= h->rbufsize)
            h->rrptr -= h->rbufsize;
    }
    return 0;
}

    
/*********************************************************************
* Function: rxpeekbuf
**********************************************************************
* In:  Serial port handle, read buffer pointer and size
* Out: Data ready flag
*
* Description:
*   Peek bytes from RX buffer.
*
*********************************************************************/
long rxpeekbuf(serialcom* h, char* buf, long bufsize) {

    long i;
    long rptr = h->rrptr;
    if (rxbytecnt(h) < bufsize)
        return 1; // Not enough data in RX buffer
    for (i = 0; i < bufsize; i++) {
        *buf++ = h->rbuf[rptr++];
        if (rptr >= h->rbufsize) rptr = 0;
    }
    return 0;
}


/*********************************************************************
* Function: rxgetchar
**********************************************************************
* In:  Serial port handle
* Out: Data byte
*
* Description:
*   Copies a byte from RX buffer.
*
*********************************************************************/
long rxgetchar (serialcom* h, char *byte) {

    if (rxbytecnt(h) < 1)
        return 1; // Not enough data in RX buffer
    if (byte != 0)
    {
        // Copy byte
        *byte = h->rbuf[h->rrptr++];
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    }
    else
    {
        // Flush byte
        h->rrptr++;
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    }
    return 0;
}


/*********************************************************************
* Function: rxgetshort
**********************************************************************
* In:  Serial port handle
* Out: 16 bits of data
*
* Description:
*   Copies 16 bits from RX buffer.
*
*********************************************************************/
short rxgetshort (serialcom* h, char byteswap) {

    short data;
    char  *ptr = (char *) &data;
 
    if (byteswap == 0) {   
        // No byte swap
        ptr[0] = h->rbuf[h->rrptr++];
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
        ptr[1] = h->rbuf[h->rrptr++];
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    } else {
        // Byte swap
        ptr[1] = h->rbuf[h->rrptr++];
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
        ptr[0] = h->rbuf[h->rrptr++];
        if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    }

    return data;
}


/*********************************************************************
* Function: rxgetlong
**********************************************************************
* In:  Serial port handle
* Out: 32 bits of data
*
* Description:
*   Copies 32 bits from RX buffer.
*
*********************************************************************/
long rxgetlong (serialcom* h) {

    long data;
    char  *ptr = (char *) &data;
    
    ptr[0] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[1] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[2] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[3] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    return data;
}


/*********************************************************************
* Function: rxgetfloat
**********************************************************************
* In:  Serial port handle
* Out: 32 bits of data (single precision floating point)
*
* Description:
*   Copies 32 bits from RX buffer.
*
*********************************************************************/
float rxgetfloat (serialcom* h) {

    float data;
    char  *ptr = (char *) &data;
    
    ptr[0] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[1] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[2] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[3] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    return data;
}


/*********************************************************************
* Function: rxgetdouble
**********************************************************************
* In:  Serial port handle
* Out: 64 bits of data (double precision floating point)
*
* Description:
*   Copies 64 bits from RX buffer.
*
*********************************************************************/
double rxgetdouble (serialcom* h) {

    double data;
    char  *ptr = (char *) &data;
    
    ptr[0] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[1] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[2] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[3] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[4] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[5] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[6] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    ptr[7] = h->rbuf[h->rrptr++];
    if (h->rrptr >= h->rbufsize) h->rrptr = 0;
    return data;
}


/*********************************************************************
* Function: rxclrbytes
**********************************************************************
* In:  Serial port handle, number of byte to clear
* Out: Void
*
* Description:
*   Clears bytes from RX buffer.
*
*********************************************************************/
void rxclrbytes (serialcom* h, unsigned long bytecnt) {

    h->rrptr += bytecnt;
    if (h->rrptr >= h->rbufsize) h->rrptr -= h->rbufsize;
}
