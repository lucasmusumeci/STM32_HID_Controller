/*
 * usb.c
 *
 *  Created on: Nov 23, 2023
 *      Author: Laurent
 */

#include "usb.h"

#define SIZE_MSG 6

#define USB_OTG_HS_INEP(i)    ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_HS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))
#define USB_OTG_HS_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_HS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (i)*USB_OTG_EP_REG_SIZE))

/*
 * Local static functions
 */

static void 	USB_Reset_Event_Handler		(void);
static void 	USB_Setup_Packet_Handler	(void);

static void 	USB_FIFO_Read				(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt);
static void 	USB_FIFO_Write				(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt);



/*
 * Global variables
 */

USB_OTG_GlobalTypeDef * const USB_OTG_HS_GLOBAL  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE);
USB_OTG_DeviceTypeDef * const USB_OTG_HS_DEVICE  = (void*)(USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE);

usb_packet_t		g_usb_packet;			// Incoming packet info
usb_setup_packet_t 	g_usb_setup_packet;		// Setup packet info

usb_in_ctrl_t 		g_usb_in_ctrl;			// IN  control Endpoint (EP0, EP1)
usb_out_ctrl_t 		g_usb_out_ctrl;			// OUT control Endpoint (EP0 only for a HID/keyboard device)

usb_keyb_t 			g_usb;					// USB CDC data structure

extern uint8_t 	g_controler_event_neutral[SIZE_MSG];

/*
 * USB HID KEYBOARD DESCRIPTORS
 */

#define BCD_DEVICE  					0x0200
#define VENDOR_ID   					0x1209			// pid.code
#define PRODUCT_ID  					0x0001

#define EP0_MAX_PACKET_SIZE				0x40			// 64 bytes
#define EP0_MAX_PACKET_SIZE_UNSIGNED    0x40U

#define EP1_MAX_PACKET_SIZE				0x40			// 64 bytes
#define EP1_MAX_PACKET_SIZE_UNSIGNED    0x40U

#define USB_DESCTYPE_DEVICE				0x01
#define	USB_DESCTYPE_CONFIGURATION		0x02
#define USB_DESCTYPE_INTERFACE			0x04
#define USB_DESCTYPE_ENDPOINT			0x05




/* USB HID KEYBOARD Device Descriptor - 18 bytes */

uint8_t devDesc[] =
{
		0x12,                  		/* 0   bLength 										 */
		USB_DESCTYPE_DEVICE,   		/* 1   bdescriptorType - Device 						 */
		0x00,0x02,                  /* 2-3 bcdUSB version in LE (=little-endian)	-> Least-significant byte first	*/
		0x00,						/* 4   bDeviceClass - 0 = Specified by interface and 255 = Vendor Specific	*/
		0x00,                  		/* 5   bDeviceSubClass  - Specified by interface 	 */
		0x00,                  		/* 6   bDeviceProtocol  - Specified by interface 	 */
		EP0_MAX_PACKET_SIZE,        /* 7   bMaxPacketSize for EP0 - max = 64 			 */
		(VENDOR_ID & 0xFF),    		/* 8   idVendor (LSByte)									 */
		(VENDOR_ID >> 8),      		/* 9   idVendor (MSByte)									 */
		(PRODUCT_ID & 0xFF),   		/* 10  idProduct (LSByte)									 */
		(PRODUCT_ID >> 8),     		/* 11  idProduct (MSByte)							 	 */
		(BCD_DEVICE & 0xFF),   		/* 12  bcdDevice 									 */
		(BCD_DEVICE >> 8),     		/* 13  bcdDevice 									 */
		0x01,                  		/* 14  iManufacturer - index of string 				 */
		0x02,                  		/* 15  iProduct  - index of string 					 */
		0x03,                  		/* 16  iSerialNumber  - index of string 				 */
		0x01                   		/* 17  bNumConfigurations 							 */
};


/* USB HID KEYBOARD String Descriptors */

// Index 0 -> Supported languages (always)
uint8_t strDesc0[]=
{
		0x04,                       		/* 0  bLength = 4 */
		0x03, 								/* 1  bDescriptortype - String */
		0x09, 0x04							/* 2  wLANGID 0x0409 = English US */
};

// Index 1 -> iManufacturer = "OpenHID"
uint8_t strDesc1[] =
{
	    2 + 2*7,   // bLength = 2 + 2*number of UTF-16 chars
	    0x03,      // bDescriptorType = STRING
	    'O', 0x00,
	    'p', 0x00,
	    'e', 0x00,
	    'n', 0x00,
	    'H', 0x00,
	    'I', 0x00,
	    'D', 0x00
};


// Index 2 -> iProduct = "Generic Gamepad"
uint8_t strDesc2[] =
{
	    2 + 2*15,   // bLength = 32 bytes
	    0x03,       // bDescriptorType = STRING
	    'G', 0x00,
	    'e', 0x00,
	    'n', 0x00,
	    'e', 0x00,
	    'r', 0x00,
	    'i', 0x00,
	    'c', 0x00,
	    ' ', 0x00,
	    'G', 0x00,
	    'a', 0x00,
	    'm', 0x00,
	    'e', 0x00,
	    'p', 0x00,
	    'a', 0x00,
	    'd', 0x00
};


// Index 3 -> iSerial = "0001"
uint8_t strDesc3[] =
{
	    2 + 2*4,  	// 10 bytes
	    0x03,     	// bDescriptorType = STRING
	    '0', 0x00,
	    '0', 0x00,
	    '0', 0x00,
	    '1', 0x00
};


/*	USB HID Controler Configuration Descriptor - 34 bytes */

uint8_t cfgDesc[] =
{
		/* Configuration Descriptor */
		0x09,                       		/* 0  bLength */
		USB_DESCTYPE_CONFIGURATION, 		/* 1  bDescriptortype - Configuration */
		0x22, 0x00,                 		/* 2  wTotalLength = 34 bytes	*/
		0x01,                       		/* 4  bNumInterfaces */
		0x01,                       		/* 5  bConfigurationValue */
		0x00,                       		/* 6  iConfiguration - index of string */
		0xC0,                       		/* 7  bmAttributes - 0x80 Bus powered ; 0xC0 Self powered ; +0x20 to add remote wkup*/
		0x32,                       		/* 8  bMaxPower in 2mA units = 100mA */

		/* Interface Descriptor */
		0x09,                       		/* 0  bLength */
		USB_DESCTYPE_INTERFACE,     		/* 1  bDescriptorType - Interface */
		0x00,                       		/* 2  bInterfaceNumber - Interface 0 */
		0x00,                       		/* 3  bAlternateSetting */
		0x01,                       		/* 4  bNumEndpoints = 1 (IN only) */
		0x03,   							/* 5  bInterfaceClass = HID */
		0x00,       						/* 6  bInterfaceSubClass 0 = Generic HID ; 1 = Boot Interface Subclass */
		0x00,								/* 7  bInterfaceProtocol 0 = None ; 1 = Keyboard ; 2 = Mouse */
		0x00,                       		/* 8  iInterface index */

		/* HID Descriptor */
		0x09,                      			/* 0  bLength */
		0x21, 								/* 1  bDescriptortype = HID */
		0x11, 0x01,                			/* 2  bcdHID = 1.11 (in LE) */
		0x00,								/* 4  bCountryCode */
		0x01,								/* 5  bNumDescriptors */
		0x22,								/* 6  bDescriptorType = Report */
		0x3D, 0x00,							/* 7  wDescriptor Length = 61 bytes */

		/* Endpoint descriptor */
		0x07,                     			/* 0    bLength */
		USB_DESCTYPE_ENDPOINT,    			/* 1    bDescriptorType = Endpoint */
		0x81,								/* 2    bEndpointAddress = IN endpoint 1 */
		0x03,                     			/* 3    bmAttributes = Interrupt */
		EP1_MAX_PACKET_SIZE, 0x00,          /* 4-5  wMaxPacketSize (in LE) */
		0x0A                      			/* 6    bInterval = 10 ms */
};


/*	USB HID KEYBOARD REPORT Descriptor - 63 bytes */

uint8_t	reportDesc[] =
{
	    0x05, 0x01,        // Usage Page (Generic Desktop)
	    0x09, 0x05,        // Usage (Gamepad)
	    0xA1, 0x01,        // Collection (Application)

	    // --- 2 Joysticks ---
	    0x09, 0x30,        // Usage X
	    0x09, 0x31,        // Usage Y
	    0x09, 0x32,        // Usage Rx
	    0x09, 0x35,        // Usage Ry
	    0x15, 0x00,        // Logical Min 0
	    0x26, 0xFF, 0x00,  // Logical Max 255 => default pos = (128;128)
		//0x15, 0x80,        // Logical Minimum = -128
		//0x25, 0x7F,        // Logical Maximum = +127
	    0x75, 0x08,        // Report Size = 8 bits
	    0x95, 0x04,        // Report Count = 4
	    0x81, 0x02,        // Input (Data,Var,Abs)

	    // --- D-Pad ---
	    0x09, 0x39,        // Usage Hat switch
	    0x15, 0x00,        // Logical Min = 0
	    0x25, 0x07,        // Logical Max = 7
	    0x35, 0x00,        // Physical Min = 0
	    0x46, 0x3B, 0x01,  // Physical Max = 315
	    0x65, 0x14,        // Unit: degrees
	    0x75, 0x04,        // Report Size = 4 bits
	    0x95, 0x01,        // Report Count = 1
	    0x81, 0x42,        // Input (Data,Var,Abs,Null)

	    // --- Buttons 1–12 ---
	    0x05, 0x09,        // Usage Page (Buttons)
	    0x19, 0x01,        // Usage Min 1
	    0x29, 0x0C,        // Usage Max 12
	    0x15, 0x00,        // Logical Min 0
	    0x25, 0x01,        // Logical Max 1
	    0x75, 0x01,        // Report Size = 1 bit
	    0x95, 0x0C,        // Report Count = 12
	    0x81, 0x02,        // Input (Data,Var,Abs)

	    0xC0               // End Collection
};


/*
 * USB Send function
 */

void BSP_USB_Send(uint8_t *msg, uint8_t length)
{
	uint8_t i;

    if (g_usb.tx_busy) return;   // previous transfer still in progress
    g_usb.tx_busy = 1;

	// Copy message into g_usb TX buffer
	for (i=0; i<length; i++)
	{
		g_usb.tx_buffer[i] = msg[i];
	}

	// Set number of bytes to transmit
	g_usb.tx_nbytes = length;

	// Setup IN EP1 for transmission
    USB_OTG_HS_INEP(1)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
    USB_OTG_HS_INEP(1)->DIEPTSIZ |= 1U <<19U;

    USB_OTG_HS_INEP(1)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// nbytes
    USB_OTG_HS_INEP(1)->DIEPTSIZ |= USB_OTG_DIEPTSIZ_XFRSIZ & g_usb.tx_nbytes;

    // Enable device IN EP1 FIFO empty (TXFE) interrupt -> This is done to wait for available space in the TX FIFO
    USB_OTG_HS_DEVICE->DIEPEMPMSK |= 1U <<1U;

    // Enable IN EP1 (automatically disabled after previous transfer completed)
    USB_OTG_HS_INEP(1)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
}



/*
 * USB Core initialization for Device Mode
 */
void BSP_USB_Core_Init()
{
	// USB_OTG_HS GPIO Configuration
	// PA4		-> USB_OTG_HS_SOF	(AF12)
	// PB12     -> USB_OTG_HS_ID	(AF12)
	// PB14     -> USB_OTG_HS_DM	(AF12)
	// PB15     -> USB_OTG_HS_DP 	(AF12)
	// PB13		-> USB_OTG_HS_VBUS

	uint32_t	i;

	// Enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Configure PA4, PB12, PB14, PB15 as AF mode
	GPIOA->MODER &= ~ GPIO_MODER_MODER4;
	GPIOB->MODER &= ~(GPIO_MODER_MODER12 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
	GPIOA->MODER |=   0x02 <<GPIO_MODER_MODER4_Pos;
	GPIOB->MODER |=  (0x02 <<GPIO_MODER_MODER12_Pos | 0x02 <<GPIO_MODER_MODER14_Pos | 0x02 <<GPIO_MODER_MODER15_Pos);

	// Set to push-pull outputs
	GPIOA->OTYPER &= ~ GPIO_OTYPER_OT4_Msk;
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT12_Msk | GPIO_OTYPER_OT14_Msk | GPIO_OTYPER_OT15_Msk);

	// Set to very high speed
	GPIOA->OSPEEDR &= ~ GPIO_OSPEEDER_OSPEEDR4;
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12 | GPIO_OSPEEDER_OSPEEDR14 | GPIO_OSPEEDER_OSPEEDR15);
	GPIOA->OSPEEDR |=   0x03 <<GPIO_OSPEEDER_OSPEEDR4_Pos;
	GPIOB->OSPEEDR |=  (0x03 <<GPIO_OSPEEDER_OSPEEDR12_Pos) | (0x03 <<GPIO_OSPEEDER_OSPEEDR14_Pos) | (0x03 <<GPIO_OSPEEDER_OSPEEDR15_Pos);

	// No pull resistors
	GPIOA->PUPDR &= ~ GPIO_PUPDR_PUPDR4_Msk;
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR12_Msk | GPIO_PUPDR_PUPDR14_Msk | GPIO_PUPDR_PUPDR15_Msk);

	// Connect to USB_OTG_HS (AF12)
	GPIOA->AFR[0] &= ~(0x000F0000);
	GPIOA->AFR[0] |=   0x000C0000;
	GPIOB->AFR[1] &= ~(0xFF0F0000);
	GPIOB->AFR[1] |=   0xCC0C0000;

	// Configure PB13 as input
	GPIOB->MODER &= ~(GPIO_MODER_MODER13);
	GPIOB->MODER |=  (0x00 <<GPIO_MODER_MODER13_Pos);


	// USB Global Core Configuration

	// Start USB_OTG_HS clock
	RCC->AHB1ENR |= RCC_AHB1ENR_OTGHSEN;

	// Global AHB USB Configuration
	// - Global USB interrupts are disabled
	// - TXFE signals that TX FIFO is completely empty
	USB_OTG_HS->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
	USB_OTG_HS->GAHBCFG |=  USB_OTG_GAHBCFG_TXFELVL;

	// NVIC enable USB_OTG_HS interrupts
	NVIC_SetPriority(OTG_HS_IRQn, 1);
	NVIC_EnableIRQ(OTG_HS_IRQn);

	// Select internal PHY interface layer (0: USB 2.0 external ULPI high-speed PHY ; 1: USB 1.1 full-speed serial mode)
	USB_OTG_HS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

	// Wait for AHB master state machine to be in IDLE state
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

	// Core soft reset
	USB_OTG_HS->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

	// Wait (again) for AHB master state machine to be in IDLE state
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

	// Deactivate power down (i.e. transceiver becomes active)
	USB_OTG_HS->GCCFG = USB_OTG_GCCFG_PWRDWN;

	// Force DEVICE mode (no matter what the ID input pin is)
	USB_OTG_HS->GUSBCFG &= ~(USB_OTG_GUSBCFG_FHMOD | USB_OTG_GUSBCFG_FDMOD);
	USB_OTG_HS->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

	// Set the turnaround time
	USB_OTG_HS->GUSBCFG &= ~USB_OTG_GUSBCFG_TRDT_Msk;
	USB_OTG_HS->GUSBCFG |= (9U << USB_OTG_GUSBCFG_TRDT_Pos);

	delay_ms(50);

	// Deactivate VBUS Sensing B
    // USB_OTG_HS->GCCFG &= ~ USB_OTG_GCCFG_VBUSBSEN;

    // Activate VBUS Sensing B
    USB_OTG_HS->GCCFG |= USB_OTG_GCCFG_VBDEN;

    // Restart the Phy Clock
    *(__IO uint32_t *)((uint32_t)USB_OTG_HS + USB_OTG_PCGCCTL_BASE) = 0U;



	// USB DEVICE mode configuration

	// Set periodic frame interval to 80%
	USB_OTG_HS_DEVICE->DCFG &= ~(USB_OTG_DCFG_PFIVL_Msk);
	USB_OTG_HS_DEVICE->DCFG |= 0x00 <<USB_OTG_DCFG_PFIVL_Pos;

	// Set to full speed
	USB_OTG_HS_DEVICE->DCFG |= 0x03 <<USB_OTG_DCFG_DSPD_Pos;

	// Flush all TX FIFOs
	USB_OTG_HS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x10 <<USB_OTG_GRSTCTL_TXFNUM_Pos);
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	// Flush RX FIFO
	USB_OTG_HS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

	// Mask all IN EP interrupts
	USB_OTG_HS_DEVICE->DIEPMSK 	= 0x00000000U;

	// Mask all OUT EP interrupts
	USB_OTG_HS_DEVICE->DOEPMSK 	= 0x00000000U;

	// Mask all EP interrupts
	USB_OTG_HS_DEVICE->DAINTMSK = 0x00000000U;


	// For all IN EP (0, 1..5)
	for (i=0; i<6; i++)
	{
		// If the EP is currently enabled
	    if ((USB_OTG_HS_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
	    {
	    	// Disable the EP and set NAK
	    	USB_OTG_HS_INEP(i)->DIEPCTL = (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);
	    }
	    else
	    {
	    	// Reset EP control register
	    	USB_OTG_HS_INEP(i)->DIEPCTL = 0x00000000U;
	    }

	    // Set IN EP size to zero
	    USB_OTG_HS_INEP(i)->DIEPTSIZ = 0x00000000U;

	    // Clear ALL IN EP interrupts flags
	    USB_OTG_HS_INEP(i)->DIEPINT |=  0x0000287BU;
	}

	// For all OUT EP (0, 1..5)
	for (i=0; i<6; i++)
	{
		// If the EP is currently enabled
		if ((USB_OTG_HS_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)	// Passe dans le else 4 fois
		{
			// Disable the EP and set NAK
			USB_OTG_HS_OUTEP(i)->DOEPCTL = (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);
	    }
	    else
	    {
	    	// Reset EP control register
	    	USB_OTG_HS_OUTEP(i)->DOEPCTL = 0x00000000U;
	    }

		// Set OUT EP size to zero
		USB_OTG_HS_OUTEP(i)->DOEPTSIZ = 0x00000000U;

		 // Clear ALL OUT EP interrupt flags
		USB_OTG_HS_OUTEP(i)->DOEPINT  |= 0x0000313BU;
	}

	// Disable all interrupts
	USB_OTG_HS->GINTMSK = 0x00000000U;

	// Clear ALL pending interrupts
	USB_OTG_HS->GINTSTS |= 0xF030FC0AU;

	// Enable the common interrupts
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

	// Enable interrupts matching to the Device mode ONLY
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM;				// USB SUSPEND event
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_USBRST;					// USB RESET event
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_ENUMDNEM;				// USB ENUMERATION done event
	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_IEPINT;					// IN EP event

	// Soft-Disconnect USB device by disabling pull-up/pull-down
	USB_OTG_HS_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
	delay_ms(10);

	// Set RX FIFO size to 0x80 = 128 words = 512 bytes
	USB_OTG_HS->GRXFSIZ = (uint32_t)0x80;

	// Set TX FIFO for IN EP0 to 0x40 = 64 words = 256 bytes
	USB_OTG_HS->DIEPTXF0_HNPTXFSIZ = ( (uint32_t)0x40 <<16U | 0x80);

	// Set TX FIFO for IN EP1 to 0x80 = 128 words = 512 bytes				--> Total = 512 + 256 + 512 = 1280 bytes = 1,25kB
	USB_OTG_HS->DIEPTXF[0] = ((uint32_t)0x80 <<16U | (0x80 + 0x40) );

	// Soft-Connect USB device by enabling pull-up/pull-down
	USB_OTG_HS_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS ;
	delay_ms(10);

	// Enable global interrupts
	USB_OTG_HS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

	// Perform Reset initializations
	USB_Reset_Event_Handler();
}




/**********************************************************************
 *                                                                    *
 * USB_OTG_HS Interrupt Handler (this is where everything is done !)  *
 *                                                                    *
 **********************************************************************
 */

void OTG_HS_IRQHandler(void)
{
	uint32_t		temp;				// Temporary register
	uint32_t 		*p_fifo;
	uint16_t		i, k;


	// Deal with USB RESET event
	if ((USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_USBRST) == USB_OTG_GINTSTS_USBRST)
	{
		// Reset USB core
		USB_Reset_Event_Handler();

	    // Clear USBRST interrupt flag
	    USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_USBRST;

	    my_printf("RESET\r\n");

	    BSP_LED_Off(LEDN_BLUE);
	    g_usb.keyb_ready = 0;
	}


	// Deal with USB SUSPEND event
	if ((USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_USBSUSP) == USB_OTG_GINTSTS_USBSUSP)
	{
		// Clear USBSUSP flag
		USB_OTG_HS->GINTSTS |= USB_OTG_GINTSTS_USBSUSP;

		// Disable the USB SUSPEND event interrupt (to avoid multiple SUSPEND interrupts)
		USB_OTG_HS->GINTMSK &= ~USB_OTG_GINTMSK_USBSUSPM;

		my_printf("SUSPEND\r\n");

	    BSP_LED_Off(LEDN_BLUE);
	    g_usb.keyb_ready = 0;
	}


	// Deal with USB ENUMERATION DONE event
    if (USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_ENUMDNE)
    {
       	// Clear ENUMDNE flag
    	USB_OTG_HS->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;

    	// Re-enable the USB SUSPEND event interrupt
    	USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM;

    	my_printf("ENUM DONE\r\n");
    }


	// Deal with data RX event
    if ((USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == USB_OTG_GINTSTS_RXFLVL) // Rx non-empty
	{
		// Read RX status
		temp = USB_OTG_HS->GRXSTSR;

		// Retrieve the number of received bytes
		g_usb_packet.bcnt 	= (uint16_t)( (temp & USB_OTG_GRXSTSP_BCNT_Msk) >>USB_OTG_GRXSTSP_BCNT_Pos);
		g_usb_packet.status = (int8_t)  ( (temp & USB_OTG_GRXSTSP_PKTSTS_Msk) >>USB_OTG_GRXSTSP_PKTSTS_Pos);
		g_usb_packet.epnum	= (int8_t)  ( (temp & USB_OTG_GRXSTSP_EPNUM_Msk) >>USB_OTG_GRXSTSP_EPNUM_Pos);

		// pop FIFO
		USB_OTG_HS->GRXSTSP;

		my_printf("PK %d on EP%d of %d bytes : ", g_usb_packet.status, g_usb_packet.epnum, g_usb_packet.bcnt);

		// If this is a received SETUP packet (8 bytes received on EP0)
		if (g_usb_packet.status == USB_PKT_STATUS_SETUP)
		{
			my_printf("[SETUP]");

			// Handle SETUP packet
			USB_Setup_Packet_Handler();

			// Clear NAK bit and set EPENA to start transmission on EP0 (this terminates the SETUP transaction)
			USB_OTG_HS_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
		}


		// If this is a SETUP transaction completed
		if (g_usb_packet.status == USB_PKT_STATUS_SETUP_COMPLETE)
		{
			my_printf("[SETUP_COMPLETE]");

			// Clear NAK bit and set EPENA to start transmission on EP0
			USB_OTG_HS_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA;
		}


		// If this is an OUT transaction
		if (g_usb_packet.status == USB_PKT_STATUS_OUT)
		{
			my_printf("[OUT]");

			switch (g_usb_packet.epnum)
			{
				case 0 :	// EP0
				{
					my_printf("[EP0]");

					// k = number of 4 bytes words
					k = (g_usb_packet.bcnt >>2U); // divide by 4
					if ( (g_usb_packet.bcnt - (k <<2U)) > 0) k++; // leftover
					my_printf(" -> RECEIVING %d bytes in %d words ", g_usb_packet.bcnt, k);


					g_usb_out_ctrl.process = 1;

					// Retrieve DATA from OUT message
					p_fifo = (uint32_t*)0x50001000;
					USB_FIFO_Read(p_fifo, g_usb.rx_buffer, g_usb_packet.bcnt);

					// Print result
					for (i=0; i<g_usb_packet.bcnt; i++)
					{
						my_printf("[%02x] ", g_usb.rx_buffer[i]);
					}

					break;
				}
			}
		}

		// If this is an OUT transaction completed
		if (g_usb_packet.status == USB_PKT_STATUS_OUT_COMPLETE)
		{
			my_printf("[OUT_COMPLETE]");

			// Switch depending on end-point number
			switch(g_usb_packet.epnum)
			{
				case 0:			// EP0
				{
					my_printf("[EP0]");

					if (g_usb_out_ctrl.process == 1)
					{
						g_usb_out_ctrl.process = 0;

						// Disable device IN EP FIFO empty interrupt
						USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);
					}
					else
					{
						// Do nothing
						my_printf(" Ignored...");
					}

					break;
				}

				case 1:			// EP1
				{
					my_printf("[EP1]");
					my_printf("Re-enabling OUT EP1 for next reception");


					// Re-enable OUT EP1
					USB_OTG_HS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;

					// Clear OUT EP1 interrupt flags
					USB_OTG_HS_OUTEP(1)->DOEPINT |= 0xFF;

					// Clear NACK bit so that ACK is ready for next reception
					USB_OTG_HS_OUTEP(1)->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;

					break;
				}
			}
		}


		// Error catching...
		if (g_usb_packet.status == 0)
		{
			my_printf(" Weird...");

			// Flush RX FIFO
			USB_OTG_HS->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
			while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);
		}

		my_printf("\r\n");
	}


	// Deal with IN EP event
    if ( (USB_OTG_HS->GINTSTS & USB_OTG_GINTSTS_IEPINT) == USB_OTG_GINTSTS_IEPINT )
	{
		// If this is for EP0
		if ( (USB_OTG_HS_DEVICE->DAINT & 0x00000001) == 0x00000001)
		{
			my_printf("[IEP0]");

			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_NAK_Msk)   		 == USB_OTG_DIEPINT_NAK)   		  my_printf("[NAK]");   	  // NAK transmitted or received
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_PKTDRPSTS_Msk)   == USB_OTG_DIEPINT_PKTDRPSTS)    my_printf("[PKTRDPSTS]");   // Packet dropped status
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_TXFIFOUDRN_Msk)  == USB_OTG_DIEPINT_TXFIFOUDRN)   my_printf("[TXFIFOUDRN]");  // Transmit FIFO underrun
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_INEPNM_Msk) 	 == USB_OTG_DIEPINT_INEPNM) 	  my_printf("[INEPNM]"); // IN received with EP mismatch

			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_XFRC_Msk)   == USB_OTG_DIEPINT_XFRC)   my_printf("[XFRC]");   // Transfer completed interrupt
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == USB_OTG_DIEPINT_EPDISD) my_printf("[EPDISD]"); // Endpoint disbaled interrupt
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_TOC_Msk)    == USB_OTG_DIEPINT_TOC)    my_printf("[TOC]");	  // Timeout condition
			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_INEPNE_Msk) == USB_OTG_DIEPINT_INEPNE) my_printf("[INEPNE]"); // IN endpoint NAK effective

			if ( (USB_OTG_HS_INEP(0)->DIEPINT & USB_OTG_DIEPINT_ITTXFE_Msk) == USB_OTG_DIEPINT_ITTXFE ) // IN token received when Tx FIFO empty
			{
				my_printf("[ITTXFE]");

				// If there's at least one packet to send
				if (g_usb_in_ctrl.pcnt > 0)
				{
					my_printf(" -> Sending %d byte in %d packet", g_usb_in_ctrl.bcnt, g_usb_in_ctrl.pcnt);

					if (g_usb_in_ctrl.bcnt <= EP0_MAX_PACKET_SIZE)
					{
						USB_FIFO_Write((uint32_t*)0x50001000, (uint8_t*)&g_usb_in_ctrl.buffer[g_usb_in_ctrl.pindex * EP0_MAX_PACKET_SIZE], g_usb_in_ctrl.bcnt);
					}

					else
					{
						USB_FIFO_Write((uint32_t*)0x50001000, (uint8_t*)&g_usb_in_ctrl.buffer[g_usb_in_ctrl.pindex * EP0_MAX_PACKET_SIZE], EP0_MAX_PACKET_SIZE);
					}

					// Decrement packet counter
					g_usb_in_ctrl.pcnt--;

					// Prepare for next packet (if necessary)
					if (g_usb_in_ctrl.pcnt > 0)
					{
						// Increment packet index
						g_usb_in_ctrl.pindex++;

						// Update byte count for next transfer
						g_usb_in_ctrl.bcnt -= EP0_MAX_PACKET_SIZE;

						// If there is only one packet, set number of bytes to transmit
						if (g_usb_in_ctrl.pcnt == 1)
						{
							USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;
						}

						// Otherwise, set byte count to maximum for first packet
						else
						{
							USB_OTG_HS_INEP(0)->DIEPTSIZ |= EP0_MAX_PACKET_SIZE;
						}

						// Enable all IN EP interrupts
						USB_OTG_HS->GINTMSK |= USB_OTG_GINTMSK_IEPINT;

						// Enable device IN EP FIFO empty interrupt
						USB_OTG_HS_DEVICE->DIEPEMPMSK |= 1U <<0U;

						// Enable IN EP0
						USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
					}
				}

				// If there's nothing to send
				else
				{
					my_printf(" -> Nothing to send");
				}


			}

			// Clear all EP0 IN interrupt flags
			//
			USB_OTG_HS_INEP(0)->DIEPINT |= 0x0000287B;

			my_printf("\r\n");
		}


		// If this is for EP1
		if ( (USB_OTG_HS_DEVICE->DAINT & 0x00000002) == 0x00000002)
		{
			my_printf("[IEP1]");
			my_printf("[%08x]", USB_OTG_HS->GINTSTS);

	    	while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
	    	USART3->TDR = '1';


	    	// Transfer completed interrupt
			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_XFRC_Msk) == USB_OTG_DIEPINT_XFRC)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'X';

		    	// Reset transfer completed interrupt mask
				USB_OTG_HS_DEVICE->DIEPMSK &= ~USB_OTG_DIEPMSK_XFRCM;

			    // Put EP1 back into NAK state until next BSP_USB_Send()
			    USB_OTG_HS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;

			    g_usb.tx_busy = 0;
			    USB_OTG_HS_INEP(1)->DIEPINT = USB_OTG_DIEPINT_XFRC;  // clear XFRC only here

				// Transmission is done
				// The EP1 has been automatically disabled
			}

			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_EPDISD_Msk) == USB_OTG_DIEPINT_EPDISD )
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'D';
			}

			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_TOC_Msk) == USB_OTG_DIEPINT_TOC)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'T';
			}

			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_INEPNE_Msk) == USB_OTG_DIEPINT_INEPNE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'N';
			}

			// IN token received when Tx FIFO is empty
			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_ITTXFE_Msk) == USB_OTG_DIEPINT_ITTXFE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'I';
			}

			if ( (USB_OTG_HS_INEP(1)->DIEPINT & USB_OTG_DIEPINT_TXFE_Msk) == USB_OTG_DIEPINT_TXFE)
			{
				while ( (USART3->ISR & USART_ISR_TC) != USART_ISR_TC);
		    	USART3->TDR = 'E';

		    	// Writing to the FIFO triggers the data transmission upon next IN token
		        if (g_usb.tx_nbytes > 0)
		        {
		            uint16_t pcnt = (g_usb.tx_nbytes + EP1_MAX_PACKET_SIZE - 1) / EP1_MAX_PACKET_SIZE;

		            // Set transfer size
		            USB_OTG_HS_INEP(1)->DIEPTSIZ  = 0;
		            USB_OTG_HS_INEP(1)->DIEPTSIZ |= (pcnt << 19U);
		            USB_OTG_HS_INEP(1)->DIEPTSIZ |= g_usb.tx_nbytes;

		            // Write data into FIFO FIRST
		            p_fifo = (uint32_t*)0x50002000;
		            USB_FIFO_Write(p_fifo, g_usb.tx_buffer, g_usb.tx_nbytes);

		            g_usb.tx_nbytes = 0;

		            // Enable XFRC interrupt to catch transfer completion
		            USB_OTG_HS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;
		        }

		        // Disable TXFE interrupt — job done
		        USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U << 1U);
			}

			// With — clear everything EXCEPT bit 0 (XFRC):
			USB_OTG_HS_INEP(1)->DIEPINT |= 0x0000287A;

			my_printf("\r\n");
		}
	}
}


/*
 * USB RESET event handler
 */
static void USB_Reset_Event_Handler(void)
{
	uint8_t		i;

	// Reset Remote Wake-up Signaling flag (don't know why...)
	USB_OTG_HS_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

	// Flush EP0 TX FIFO
	USB_OTG_HS->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (0x00 <<USB_OTG_GRSTCTL_TXFNUM_Pos);
	while ((USB_OTG_HS->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);

	// Clear all EP Interrupt flags
    for (i=0; i<6; i++)
    {
    	USB_OTG_HS_INEP(i)->DIEPINT  |= 0x0000287BU;
    	USB_OTG_HS_OUTEP(i)->DOEPINT |= 0x0000313BU;
    }

    // Un-mask Interrupt for EP0 (IN/OUT)
    USB_OTG_HS_DEVICE->DAINTMSK |= 0x00010001U;

	// Un-mask Interrupt for OUT EP : SETUP | Transfet Compete | EP disable
    USB_OTG_HS_DEVICE->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM | USB_OTG_DOEPMSK_EPDM);

	// Set Default Address to 0
    USB_OTG_HS_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

	// Setup EP0 to receive SETUP messages
    USB_OTG_HS_OUTEP(0)->DOEPTSIZ = 0U;
    USB_OTG_HS_OUTEP(0)->DOEPTSIZ |= (USB_OTG_DOEPTSIZ_PKTCNT & (1U << 19U)) ;
    USB_OTG_HS_OUTEP(0)->DOEPTSIZ |= (3U * 8U);										// 3 packets of 8 bytes
    USB_OTG_HS_OUTEP(0)->DOEPTSIZ |=  USB_OTG_DOEPTSIZ_STUPCNT;  					// 3 packets
}


/*
 * USB SETUP packet handler
 */
static void USB_Setup_Packet_Handler(void)
{
	uint8_t		rx_data[EP0_MAX_PACKET_SIZE];		// Buffer to store incoming packet data

	// Retrieve DATA from SETUP message
	USB_FIFO_Read((uint32_t *)0x50001000, rx_data, g_usb_packet.bcnt);

	// Parse SETUP packet
	g_usb_setup_packet.bmRequestType = rx_data[0];
	g_usb_setup_packet.bRequest		 = rx_data[1];
	g_usb_setup_packet.wValue		 = rx_data[3] <<8U | rx_data[2];
	g_usb_setup_packet.wIndex		 = rx_data[5] <<8U | rx_data[4];
	g_usb_setup_packet.wLength		 = rx_data[7] <<8U | rx_data[6];


	/* STANDARD REQUESTS */

	if ( (g_usb_setup_packet.bmRequestType == 0x00) || (g_usb_setup_packet.bmRequestType == 0x80) )
	{
		// Switch depending on request
		switch(g_usb_setup_packet.bRequest)
		{
			// 														GET DESCRIPTOR
			case USB_REQUEST_GET_DESCRIPTOR:
			{
				my_printf("[GET DESCRIPTOR]");

				// Switch depending on requested descriptor type (MSB of wValue)
				switch((g_usb_setup_packet.wValue & 0xFF00) >>8U)
				{
					//														DEVICE
					case USB_DESCRIPTOR_DEVICE:
					{
						my_printf("[DEVICE]");

						// Setup IN control structure for Device Descriptor sending
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 1;
						g_usb_in_ctrl.bcnt   = sizeof(devDesc);			// 18 bytes
						g_usb_in_ctrl.pindex = 0;
						g_usb_in_ctrl.buffer = (uint8_t*)devDesc;


						break;
					}

					//													CONFIGURATION
					case USB_DESCRIPTOR_CONFIGURATION:
					{
						my_printf("[CONFIGURATION %d]", g_usb_setup_packet.wLength);

						// Setup IN control structure for Configuration Descriptor sending

						// If the number of bytes is specified and below 64
						if (g_usb_setup_packet.wLength <EP0_MAX_PACKET_SIZE)
						{
							g_usb_in_ctrl.epnum  = 0;
							g_usb_in_ctrl.pcnt   = 1;
							g_usb_in_ctrl.bcnt   = g_usb_setup_packet.wLength;
							g_usb_in_ctrl.pindex = 0;
							g_usb_in_ctrl.buffer = (uint8_t*)cfgDesc;
						}

						// Else, if the number of bytes is not specified (=255)
						else
						{
							g_usb_in_ctrl.epnum  = 0;
							g_usb_in_ctrl.pcnt   = 1;
							g_usb_in_ctrl.bcnt   = 34;
							g_usb_in_ctrl.pindex = 0;
							g_usb_in_ctrl.buffer = (uint8_t*)cfgDesc;
						}

						break;
					}

					//														STRING
					case USB_DESCRIPTOR_STRING:
					{
						my_printf("[STRING %d]", (uint8_t)(g_usb_setup_packet.wValue & 0x00FF));

						// Setup IN control structure for Configuration Descriptor sending
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 1;
						g_usb_in_ctrl.pindex = 0;

						// Switch depending on STRING index
						switch ((uint8_t)(g_usb_setup_packet.wValue & 0x00FF))
						{
							case 0:						// STRING DESCRIPTOR [0]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc0))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc0);
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc0;

								break;
							}

							case 1:						// STRING DESCRIPTOR [1]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc1))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc1);
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc1;

								break;
							}

							case 2:						// STRING DESCRIPTOR [2]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc2))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc2);
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc2;

								break;
							}

							case 3:						// STRING DESCRIPTOR [3]
							{
								if (g_usb_setup_packet.wLength < sizeof(strDesc3))
								{
									g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
								}
								else
								{
									g_usb_in_ctrl.bcnt   = sizeof(strDesc3);
								}

								g_usb_in_ctrl.buffer = (uint8_t*)strDesc3;

								break;
							}
						}

						break;
					}

					case USB_DESCRIPTOR_QUALIFIER:
					{
						my_printf("[QUALIFIER]");

						// Send STALL because we are NOT a high-speed device
						g_usb_in_ctrl.epnum  = 0;
						g_usb_in_ctrl.pcnt   = 0;
						g_usb_in_ctrl.bcnt	 = 0;
						g_usb_in_ctrl.pindex = 0;
						g_usb_in_ctrl.buffer = 0x00000000;

						USB_OTG_HS_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_STALL;

						USB_OTG_HS_OUTEP(0)->DOEPCTL |= USB_OTG_DIEPCTL_STALL;

						//return; // Don't prepare EP0 for transmission
						break;
					}
				}

				// Prepare IN EP0 for transmission
				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
				USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number

				// If there is only one packet, set number of bytes to transmit
				if (g_usb_in_ctrl.pcnt == 1)
				{
					USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;
				}

				// Otherwise, set byte count to maximum for first packet
				else
				{
					USB_OTG_HS_INEP(0)->DIEPTSIZ |= EP0_MAX_PACKET_SIZE;
				}

				// Enable device IN EP FIFO empty interrupt
				USB_OTG_HS_DEVICE->DIEPEMPMSK |= 1U <<0U;

				// Enable IN EP0
				USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

				break;
			}

			//													SET ADDRESS
			case USB_REQUEST_SET_ADDRESS:
			{
				// Address is LSB of wValue
				my_printf("[SET ADRESS %d]", (uint8_t)(g_usb_setup_packet.wValue & 0x00FF));

				// Set Device address
				USB_OTG_HS_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD_Msk;
				USB_OTG_HS_DEVICE->DCFG |= (uint8_t)(g_usb_setup_packet.wValue & 0x00FF) <<USB_OTG_DCFG_DAD_Pos;

				g_usb_in_ctrl.pcnt   = 1;
				g_usb_in_ctrl.bcnt   = 0;

				// Setup IN EP0 for transmission
				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
				USB_OTG_HS_INEP(0)->DIEPTSIZ |= 1U <<19U;

				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
				USB_OTG_HS_INEP(0)->DIEPTSIZ |= 0;

				// Disable device IN EP FIFO empty interrupt
				USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

				// Enable IN EP0
				USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);

				break;
			}


			// 													SET CONFIGURATION
			case USB_REQUEST_SET_CONFIGURATION:
			{
				my_printf("[SET CONFIGURATION]");

				/*
				 *  Configure IN EP1
				 */

				// Clear all IN EP(1) interrupts flags
				USB_OTG_HS_INEP(1)->DIEPINT |=  0x0000287BU;

				// Enable all IN EP Transfer Complete (XFRC) interrupt (this one only)
				USB_OTG_HS_DEVICE->DIEPMSK  &= ~0x0000207BU;
				// USB_OTG_HS_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_XFRCM;

			    // Un-mask general interrupts from IN EP1
			    USB_OTG_HS_DEVICE->DAINTMSK |= 0x00000002U;

			    // Setup IN EP1 in INTERRUPT mode (without enabling it at that time)
				USB_OTG_HS_INEP(1)->DIEPCTL = 0x00000000U;
				USB_OTG_HS_INEP(1)->DIEPCTL |= 3 <<USB_OTG_DIEPCTL_EPTYP_Pos;			// Interrupt
				USB_OTG_HS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_USBAEP;					// USB Active
				USB_OTG_HS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;			// Set DATA0 PID
				USB_OTG_HS_INEP(1)->DIEPCTL |= 1U  << USB_OTG_DIEPCTL_TXFNUM_Pos;		// FIFO Number
				USB_OTG_HS_INEP(1)->DIEPCTL |= EP1_MAX_PACKET_SIZE_UNSIGNED << USB_OTG_DIEPCTL_MPSIZ_Pos;		// Max packet size = 64B
				//USB_OTG_HS_INEP(1)->DIEPCTL |= USB_OTG_DIEPCTL_CNAK;					// Clear NAK

				/*
				 *  Send a status IN packet on EP0
				 */

				g_usb_in_ctrl.pcnt   = 1;
				g_usb_in_ctrl.bcnt   = 0;

				// Setup IN EP0 for transmission
				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// 1 packet
				USB_OTG_HS_INEP(0)->DIEPTSIZ |= 1U <<19U;

				USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// 0 bytes
				USB_OTG_HS_INEP(0)->DIEPTSIZ |= 0;

				// Disable device IN EP FIFO empty interrupt
				USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

				// Enable IN EP0
				USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);


				// Now the keyboard is ready
				BSP_LED_On(LEDN_BLUE);
				g_usb.keyb_ready = 1;
				//BSP_USB_Send(g_controler_event_neutral, SIZE_MSG);

			    break;
			}

			default:
			{
				my_printf("[UNKNOWN SETUP MESSAGE]");
			}
		}
	}


	/* CLASS SPECIFIC REQUESTS */


	 if ((g_usb_setup_packet.bmRequestType == 0x81) && (g_usb_setup_packet.bRequest == 0x06))
	 {
		 // 																	REPORT DESCRIPTOR REQUEST
		 if (((g_usb_setup_packet.wValue & 0xFF00) >> 8U) == 0x22)
		 {
			 my_printf("[GET REPORT DESCRIPTOR]");

			 g_usb_in_ctrl.epnum  = 0;
			 if(g_usb_setup_packet.wLength < sizeof(reportDesc))
			 {
				 g_usb_in_ctrl.pcnt = (g_usb_setup_packet.wLength + EP0_MAX_PACKET_SIZE - 1) / EP0_MAX_PACKET_SIZE;
				 g_usb_in_ctrl.bcnt = g_usb_setup_packet.wLength;
			 }
			 else
			 {
				 g_usb_in_ctrl.pcnt = (sizeof(reportDesc) + EP0_MAX_PACKET_SIZE - 1) / EP0_MAX_PACKET_SIZE;
				 g_usb_in_ctrl.bcnt = sizeof(reportDesc);
			 }
			 g_usb_in_ctrl.pindex = 0;
			 g_usb_in_ctrl.buffer = (uint8_t*)reportDesc;

			 // Prepare IN EP0 for transmission
			 USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
			 USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

			 USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number
			 USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;

			 // Enable device IN EP FIFO empty interrupt
			 USB_OTG_HS_DEVICE->DIEPEMPMSK |= 1U <<0U;

			 // Enable IN EP0
			 USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
		 }
	 }


	if ((g_usb_setup_packet.bmRequestType == 0x21) && (g_usb_setup_packet.bRequest == 0x0A))
	{

		// 																SET IDLE

		my_printf("[SET IDLE]");

		// Setup IN control structure for USB CDC Line Coding
		g_usb_in_ctrl.epnum  = 0;
		g_usb_in_ctrl.pcnt   = 1;
		g_usb_in_ctrl.bcnt   = 0;
		g_usb_in_ctrl.pindex = 0;

		// Prepare IN EP0 for transmission
		USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
		USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

		USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number
		USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;

		// Disable device IN EP FIFO empty interrupt
		USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

		// Enable IN EP0
		USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}

	if ((g_usb_setup_packet.bmRequestType == 0x21) && (g_usb_setup_packet.bRequest == 0x09))
	{

		// 																SET REPORT

		my_printf("[SET REPORT]");

		// Setup IN control structure for USB CDC Line Coding
		g_usb_in_ctrl.epnum  = 0;
		g_usb_in_ctrl.pcnt   = 1;
		g_usb_in_ctrl.bcnt   = 0;
		g_usb_in_ctrl.pindex = 0;

		// Prepare IN EP0 for transmission
		USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);		// packets number
		USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt <<19U;

		USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);		// bytes number
		USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.bcnt;

		// Disable device IN EP FIFO empty interrupt
		USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U <<0U);

		// Enable IN EP0
		USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}

	// SET_PROTOCOL (bmRequestType=0x21, bRequest=0x0B)
	if ((g_usb_setup_packet.bmRequestType == 0x21) && (g_usb_setup_packet.bRequest == 0x0B))
	{
	    my_printf("[SET PROTOCOL %d]", (uint8_t)(g_usb_setup_packet.wValue & 0xFF));
	    // wValue: 0 = Boot Protocol, 1 = Report Protocol
	    // Just ACK it — we only support Report Protocol anyway

	    g_usb_in_ctrl.epnum  = 0;
	    g_usb_in_ctrl.pcnt   = 1;
	    g_usb_in_ctrl.bcnt   = 0;
	    g_usb_in_ctrl.pindex = 0;

	    // Prepare IN EP0 for zero-length status
	    USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_PKTCNT);
	    USB_OTG_HS_INEP(0)->DIEPTSIZ |= g_usb_in_ctrl.pcnt << 19U;
	    USB_OTG_HS_INEP(0)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_XFRSIZ);
	    USB_OTG_HS_INEP(0)->DIEPTSIZ |= 0;

	    // Disable FIFO empty interrupt (no data to send)
	    USB_OTG_HS_DEVICE->DIEPEMPMSK &= ~(1U << 0U);

	    // ACK with zero-length IN packet
	    USB_OTG_HS_INEP(0)->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA);
	}

}





/*
 * Read a number bytes from RX FIFO and store into buffer
 */
static void USB_FIFO_Read(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt)
{
	uint16_t	i,j,k;

	// Read FIFO
	if (bcnt > 0)
	{
		// Compute k, the number of 32-bit words to read
		k = (bcnt >>2U);
		if ((bcnt - (k <<2U)) > 0) k++;

		// Read words from DFIFO
		// (The USB core manages DFIFO addressing, so only the access register address is needed)
		j = 0;
		for(i=0; i<k; i++)
		{
			*(uint32_t*)&buffer[j] = *fifo;
			j += 4;
		}
	}
}


/*
 * Write a number of bytes from buffer into TX FIFO
 */
static void USB_FIFO_Write(uint32_t* fifo, uint8_t* buffer, uint16_t bcnt)
{
	uint16_t	i;

	// Write FIFO
	for (i=0; i<bcnt; i+=4)
	{
		*fifo = *(uint32_t*)&buffer[i];
	}
}

