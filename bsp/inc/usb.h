/*
 * usb.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Laurent
 */

#ifndef SRC_USB_H_
#define SRC_USB_H_


#include "main.h"



/*
 * Typedefs
 */

/* USB Packet structure and definitions */
typedef struct
{
	uint8_t		status;					// Packet status
	uint8_t		epnum;					// End-point number
	uint8_t		dpid;					// Data ID (0 or 1)
	uint16_t	bcnt;					// Byte count
} usb_packet_t;

#define	USB_PKT_STATUS_G_OUT_NACK		1
#define	USB_PKT_STATUS_OUT				2
#define	USB_PKT_STATUS_OUT_COMPLETE		3
#define	USB_PKT_STATUS_SETUP_COMPLETE	4
#define	USB_PKT_STATUS_SETUP			6


/* USB SETUP packet structure and definitions */
typedef struct
{
	uint8_t		bmRequestType;
	uint8_t		bRequest;
	uint16_t	wValue;
	uint16_t	wIndex;
	uint16_t	wLength;

} usb_setup_packet_t;

#define USB_REQUEST_GET_STATUS			0
#define	USB_REQUEST_CLEAR_FEATURE		1
#define USB_REQUEST_SET_FEATURE			3
#define USB_REQUEST_SET_ADDRESS			5
#define	USB_REQUEST_GET_DESCRIPTOR		6
#define USB_REQUEST_SET_DESCRIPTOR		7
#define	USB_REQUEST_GET_CONFIGURATION	8
#define USB_REQUEST_SET_CONFIGURATION	9
#define	USB_REQUEST_GET_INTERFACE		10
#define USB_REQUEST_SET_INTERFACE		11
#define USB_REQUEST_SYNCH_FRAME			12

#define USB_DESCRIPTOR_DEVICE			1
#define USB_DESCRIPTOR_CONFIGURATION	2
#define USB_DESCRIPTOR_STRING			3
#define USB_DESCRIPTOR_INTERFACE		4
#define USB_DESCRIPTOR_ENDPOINT			5
#define USB_DESCRIPTOR_QUALIFIER		6
#define USB_DESCRIPTOR_SPEED			7


/* IN Control EP structure */
typedef struct
{
	uint8_t		epnum;		// end-point number
	uint8_t		pcnt;		// packets count
	uint16_t	bcnt;		// bytes count
	uint8_t		pindex;		// packet index
	uint8_t*	buffer;		// address of byte array to be sent
} usb_in_ctrl_t;


/* OUT Control EP structure */
typedef struct
{
	uint8_t		process;	// Boolean to indicate whether an OUT transaction is issued
	uint8_t		epnum;		// end-point number
} usb_out_ctrl_t;


/* USB CDC data IN/OUT structure */
typedef struct
{
	uint8_t rx_buffer[64];		// Buffer for incoming (OUT) bytes
	uint8_t	rx_nbytes;			// Number of bytes to read
	uint8_t tx_buffer[64];		// Buffer for outgoing (IN)  bytes
	uint8_t	tx_nbytes;			// Number of bytes to send
	uint8_t	keyb_ready;			// Flag for keyboard ready (set after SET CONFIGURATION is received)
} usb_keyb_t;


/* Export global variable */
extern usb_keyb_t g_usb;


/*
 * USB API functions
 */

void BSP_USB_Core_Init	(void);
void BSP_USB_Send		(uint8_t *msg, uint8_t length);








#endif /* SRC_USB_H_ */
