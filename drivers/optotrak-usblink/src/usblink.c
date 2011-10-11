/*****************************************************************
Name:               usblink.c

Description:
    This module implements the the 6 low level driver routines for the
    PCI Interface Card for the Optotrak USB interface in Linux:

        LinkOpen()
        LinkClose()
        LinkRead()
        LinkWrite()
        LinkStatus()
        LinkReset()

Modifications:
    06/2011 - Ported driver from Windows to Linux
        Andy Barry <abarry@csail.mit.edu>

*****************************************************************/

/*****************************************************************
C Library Files Included
*****************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
//#include <windows.h>
#include <unistd.h> // for usleep()

/*****************************************************************
ND Library Files Included
*****************************************************************/
#include <ndtypes.h>
#include <ndpack.h>
#include <ndopto.h>

#include <libusb.h>
#include <string.h>
//#include <nderror.h>

/*****************************************************************
Project Files Included
*****************************************************************/
//#ifdef HOST_WIN32
//#include <windows.h>
#include "ftd2xx.h"
//#else
//#error "This file must be compiled for Windows 32"
//#endif /* HOST_WIN32 */

/*****************************************************************
Application Files Included
*****************************************************************/

/*****************************************************************
Defines
*****************************************************************/
#define MAX_DELAY 50000

/*****************************************************************
External Variables and Routines
*****************************************************************/


/*****************************************************************
Internal Routines
*****************************************************************/
void    Pause( float fSec );

/*****************************************************************
Global Variables
*****************************************************************/

static char
	szSrvrName[256],
   *szNDErrorTitle = "Northern Digital OAPI Error";
static FT_HANDLE  s_hndl;
static libusb_context *s_libucontext;
static libusb_device_handle *s_libuhandle;

#define DEFAULT_USB_TYPE	5
#define DEFAULT_USB_ID		0x403DA77
#define DEFAULT_USB_LATENCY	0 // the driver says this is a minimum of 2, but it works much better when set to 0.
#define DEFAULT_RESET_DELAY	1000

#define OPTOTRAK_USB_DESCRIPTION "Optotrak USB Interface"
#define OPTOTRAK_VENDOR_ID 0x0403
#define OPTOTRAK_PRODUCT_ID 0xda77

static int
	s_nOpenCount = 0,
	s_nUsbType      = DEFAULT_USB_TYPE,
	s_nUsbId        = DEFAULT_USB_ID,
	s_nUsbLatency   = DEFAULT_USB_LATENCY,
	s_nResetDelay   = DEFAULT_RESET_DELAY,
	s_nReadTimeout  = 10000,
	s_nWriteTimeout = 10000;

static AppParmInfo
    grUSBParms[] =
    {
        {
            "Type",
            &s_nUsbType,
            sizeof( s_nUsbType ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            0,
            0
        },
        {
            "Id",
            &s_nUsbId,
            sizeof( s_nUsbId ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            1,
            99999
        },
		{
            "Latency",
            &s_nUsbLatency,
            sizeof( s_nUsbLatency ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            2,
            255
        },
		{
            "ResetDelay",
            &s_nResetDelay,
            sizeof( s_nResetDelay ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            1,
            30000
        },
		{
            "ReadTimeOut",
            &s_nReadTimeout,
            sizeof( s_nReadTimeout ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            1,
            65535
        },
		{
            "WriteTimeOut",
            &s_nWriteTimeout,
            sizeof( s_nWriteTimeout ),
            APP_PARM_INT | APP_PARM_OPTIONAL,
            1,
            65535
        },
		{ NULL, 0 }
    };

/*****************************************************************
Static Routines
*****************************************************************/

/*****************************************************************
Name:               Pause

Input Values:
    float
        fSec        :Number of seconds to delay.

Output Values:
	None.

Return Value:
	None.

Description:
    Delay processing for the input number of seconds.

*****************************************************************/

void Pause( float fSec )
{
   clock_t end;

   end  = (clock_t)fSec * CLOCKS_PER_SEC + clock();
   while( clock() < end )
      ;
}


static FT_HANDLE OpenUSBDevice( int nType, int nId )
{

	FT_HANDLE	hndl = 0;	
	unsigned 	i;
	DWORD		dwDevs;
	FT_DEVICE_LIST_INFO_NODE *pNodes = 0;
	
	// set custom vendor and product IDs
	DWORD dwIVD = 0x0403;
    DWORD dwPID = 0xda77;
    
    DWORD err;
    
    err = FT_SetVIDPID(dwIVD, dwPID);
    if (err != FT_OK)
    {
        printf("Failed to set custom vendor and product IDs. Quitting. Error code = %d\n", err);
        goto cleanup;
    }
    
    // we need to open the device in two drivers -- all the functions use the FTDI driver except for LinkReset because the
    // FTDI driver does not implement reset, so we also open the device in libusb which does implement usb reset

    // open with libusb driver
    
    // open the libusb context
    libusb_init(&s_libucontext);

    //libusbH = libusb_open_device_with_vid_pid(context, 0x0403,0xda77);

    libusb_device **list;

    ssize_t ret;

    ret = libusb_get_device_list (s_libucontext, &list);

    if (ret == LIBUSB_ERROR_NO_MEM || ret < 1)
    {
        printf("Failed to find device in the libusb context.\n");
        goto cleanup;
    }

    int j;
    int libusb_flag = 0;
    
    for (j=0; j<ret; j++)
    {
        libusb_device *this_device = list[j];
        struct libusb_device_descriptor desc;
        
        if (libusb_get_device_descriptor(this_device, &desc) == 0)
        {
            // before attempting to open the device and get it's description, look at the vendor and product IDs
            // this also means that we won't attempt to open a bunch of devices we probably don't have permissions to open
            // furthermore, if the user hasn't set up permissions correctly, libusb will print out an error about the permissions
            // giving a good hint at what to do
            if (desc.idVendor == OPTOTRAK_VENDOR_ID && desc.idProduct == OPTOTRAK_PRODUCT_ID)
            {
                
                libusb_device_handle *udev;
                if (libusb_open(this_device, &udev) == 0)
                {
                    char buffer[256];
                    libusb_get_string_descriptor_ascii(udev, desc.iProduct, &buffer, sizeof(buffer));
                    
                    // check to make sure the description is correct
                    if (strcmp(buffer, OPTOTRAK_USB_DESCRIPTION) == 0)
                    {
                        s_libuhandle = udev;
                        libusb_flag = 1;
                    } else {
                        // wrong device
                        libusb_close(udev);
                    }
               }
           }
       }
    }
    
    // cleanup libusb list
    libusb_free_device_list(list, 1);

    if (libusb_flag == 0)
    {
        printf("libusb failed to find the Optotrak USB device.  Are you sure you have set the udev permissions correctly?\n");
        goto cleanup;
    }

    // open with FTDI driver

    if( FT_CreateDeviceInfoList( &dwDevs) == FT_OK )
    {

	    pNodes = (FT_DEVICE_LIST_INFO_NODE *)calloc( sizeof(FT_DEVICE_LIST_INFO_NODE) , dwDevs );
	    if( FT_GetDeviceInfoList( pNodes, &dwDevs ) == FT_OK )
	    {
		    for( i = 0; i < dwDevs; i++ )
		    {
			    if( (int)pNodes[i].Type == nId || // the 64-bit Linux driver seems to have types and IDs mixed up, so check for that
			        ( (int)pNodes[i].Type == nType && pNodes[i].ID == nId ) ) // this is the correct check, which works in the 32-bit linux driver
			    {
			        // the description is not returning correctly, so use a hard coded one for the linux driver
    				//err = FT_OpenEx( pNodes[i].Description, FT_OPEN_BY_DESCRIPTION, &hndl );
    				err = FT_OpenEx( OPTOTRAK_USB_DESCRIPTION, FT_OPEN_BY_DESCRIPTION, &hndl ); 
				    if( err != FT_OK )
				    {
					    hndl = 0;
					    printf("Failed to open USB device. Error code = %d\n", err);
					    goto cleanup;
				    } /* if */
				
				
				    if( FT_SetLatencyTimer( hndl, s_nUsbLatency ) != FT_OK ||
					    FT_SetTimeouts( hndl, s_nReadTimeout, s_nWriteTimeout ) != FT_OK )
				    {
				        printf("Failed to set latency timer or timeouts.  Error code = %d\n", err);
					    FT_Close( s_hndl );
					    hndl = 0;
					    goto cleanup;
				    }
				
				    // succeeded in opening the device in the FTDI context!

				    break;
			    } /* if */
		    } /* for */
	    } /* if */
	
    } /* if */
    else {
        printf("Failed to get USB device info list.\n");
        goto cleanup;
    }


cleanup:
	if( pNodes )
		free( pNodes );

	return hndl;
}

/***************************************************************************

Routine:           LinkOpen

Inputs:
    None

Returns:
    boolean               : TRUE if link correctly initialized.

Description:
    Initializes the OPTOTRAK communications link:
        Determines the address of the PC interface card
        Initializes the C012 chip.

***************************************************************************/

NDI_DECL1 boolean NDI_DECL2 LinkOpen( void )
{
//	boolean
//		bRet = FALSE;

	/*
	 * check to see if the link is open
	 */
	if( s_nOpenCount == 0 )
	{
	    if( !ReadAppParms( "optotrak", "USB Options", grUSBParms ) )
	    {
	        printf("ReadAppParms issue.\n");
			return FALSE;
		} /* if */
		s_hndl = OpenUSBDevice( s_nUsbType, s_nUsbId );
		if( !s_hndl )
		{
		    printf("PING BAD\n");
			return FALSE;
		} /* if */
    } /* if */
    
	s_nOpenCount++;
#ifdef MESSAGE_BOX_OUTPUT
	{
		char szBuff[ 100 ];
		sprintf( szBuff, "LinkOpen: Count = %d", s_nOpenCount );
		MessageBox( 0, szBuff, "LinkOpen", MB_OK );
	}
#endif
    
	return TRUE;
}

/***************************************************************************

Routine:           LinkWrite

Inputs:
    void *pBuff           : Pointer to data buffer.
    unsigned uBytes       : Number of bytes to send to the link.

Returns:
    unsigned              : Number of bytes actually written to the link.

Description:
    Attempts to send "uBytes" bytes of data starting at address "pBuff"
    to the OPTOTRAK.

***************************************************************************/

NDI_DECL1 unsigned int NDI_DECL2 LinkWrite( void *pBuff, unsigned int uBytes )
{
	DWORD	dwWritten;

	if( s_hndl )
	{
	    while (uBytes > 0)
	    {
		    if( FT_Write( s_hndl, pBuff, uBytes, &dwWritten ) == FT_OK )
		    {
			    return dwWritten;
		    } else {
		        printf("Warning: USB write failed (FT_Write failed).\n");
		    }
		}
	} /* if */
	else {
	    printf("Warning: bad handle on USB link write.\n");
	}
	return 0;
}

/***************************************************************************

Routine:           LinkRead

Inputs:
    void *pData           : Pointer to a buffer of at least "uSize" bytes
    unsigned uSize        : Number of bytes to read into the buffer.

Returns:
    unsigned              : Number of bytes actually read into the buffer.

Description:
    Attempts to read a "uSize" byte packet of data from the OPTOTRAK.  The
    data is stored in the buffer specified by "pData".

***************************************************************************/

NDI_DECL1 unsigned int NDI_DECL2 LinkRead( void *pData, unsigned int uSize )
{
	DWORD	dwRead;

	if( s_hndl )
	{
		if( FT_Read( s_hndl, pData, uSize, &dwRead ) == FT_OK )
			return dwRead;
	} /* if */
	return 0;
}

/***************************************************************************

Routine:           LinkClose

Inputs:
    None

Returns:
    void

Description:
    Closes the communications channel to the OPTOTRAK.

***************************************************************************/
NDI_DECL1 boolean NDI_DECL2 LinkClose( void )
{
#ifdef MESSAGE_BOX_OUTPUT
	{
		char szBuff[ 100 ];
		sprintf( szBuff, "LinkClose: Count = %d", s_nOpenCount );
		MessageBox( 0, szBuff, "LinkClose", MB_OK );
	}
#endif

	/* 
	 * Close regardless of the Open Count since there's no need to keep the connection open.
	 * Note that we were originally keeping the connection open until Open Count was 1 but
	 * for some unknown reason, s_hndl and s_nOpenCount were being reset to 0 between the 
	 * Close and Open count (resulting in Open errors).
	 */
	if( s_hndl )
	{
		FT_Close( s_hndl );
		s_hndl = 0;
	}

	if( s_nOpenCount > 0 )
		s_nOpenCount = 0;

    return TRUE;
}

/***************************************************************************

Routine:           LinkStatus

Inputs:
    boolean *pbReadable   : pointer to storage for Readable status
    boolean *pbWriteable  : pointer to storage for Writeable status

Returns:
    void

Description:
    Determines whether or not the OPTOTRAK system is ready to accept data
    and whether there is data available from the OPTOTRAK.

***************************************************************************/
NDI_DECL1 boolean NDI_DECL2 LinkStatus( boolean *pbReadable, boolean *pbWriteable )
{
	DWORD	dwRxStat;
	DWORD	dwTxStat;
	DWORD	dwEvStat;

	*pbReadable = *pbWriteable = FALSE;
	if( s_hndl )
	{
		if( FT_GetStatus( s_hndl, &dwRxStat, &dwTxStat, &dwEvStat ) == FT_OK )
		{
			*pbReadable = (dwRxStat > 0) ? TRUE : FALSE;
			*pbWriteable = (dwTxStat > 0) ? FALSE : TRUE;
			return TRUE;
		} /* if */
	} /* if */
    return FALSE;
}

/***************************************************************************

Routine:           LinkReset

Inputs:
    None

Returns:
    void

Description:
    Resets the OPTOTRAK system.

***************************************************************************/
NDI_DECL1 boolean NDI_DECL2 LinkReset( void )
{
	if( s_hndl )
	{
	
	    // FT_ResetPort is not implemented in linux (it runs, but doesn't do anything)
	    // we can do a similar reset using the libusb driver
	    
		//FT_ResetPort( s_hndl );
		
		libusb_reset_device(s_libuhandle);

		libusb_close(s_libuhandle);
		
		libusb_exit(s_libucontext);
		

		FT_Close( s_hndl );
		usleep( s_nResetDelay * 1000 );

		s_hndl = OpenUSBDevice( s_nUsbType, s_nUsbId );
		if( s_hndl )
		{
			return TRUE;
		}
	}
	return FALSE;
}

