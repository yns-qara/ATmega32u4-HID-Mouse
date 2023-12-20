#include "Mouse.h"

#define X_SCALE_FACTOR 10
#define Y_SCALE_FACTOR 

#define LEFT_BUTTON 1
#define RIGHT_BUTTON 4

#define JOY_X_PIN PF7 // Analog pin A0
#define JOY_Y_PIN PF6 // Analog pin A1
#define JOY_BTN_PIN PD6
#define LED_PIN PC7 

#define out_low(port,pin)         (port) &= ~(1 << (pin))
#define out_high(port,pin)        (port) |= (1 << (pin))
#define  calc(adc)  (int) 5*(adc-511)/512

void initButtons(void){
	DDRD &= ~(1 << LEFT_BUTTON);
  	PORTD |= (1 << LEFT_BUTTON);

  	DDRD &= ~(1 << RIGHT_BUTTON);
  	PORTD |= (1 << RIGHT_BUTTON);

	DDRD &= ~(1 << JOY_BTN_PIN);
  	PORTD |= (1 << JOY_BTN_PIN);
}

void InitADC(void) {
  // Select Vref=AVcc
  ADMUX |= (1 << REFS0);
  // Set prescaler to 128 and enable ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
}

uint16_t ReadADC(uint8_t ADCchannel) {
  // Select ADC channel with safety mask
  ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
  // Single conversion mode
  ADCSRA |= (1 << ADSC);
  // Wait until ADC conversion is complete
  while (ADCSRA & (1 << ADSC));
  return ADC;
}


/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevMouseHIDReportBuffer[sizeof(USB_MouseReport_Data_t)];

USB_ClassInfo_HID_Device_t Mouse_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_Mouse,
				.ReportINEndpoint             =
					{
						.Address              = MOUSE_EPADDR,
						.Size                 = MOUSE_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevMouseHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevMouseHIDReportBuffer),
			},
	};

int main(void)
{
	SetupHardware();
	InitADC();
	initButtons();
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	// Set the LED pin as output
  	DDRC |= (1 << LED_PIN);

	DDRD &= ~(1 << JOY_BTN_PIN);
  	// Enable the internal pull-up resistor for the button
  	PORTD |= (1 << JOY_BTN_PIN);

	for (;;)
	{
		HID_Device_USBTask(&Mouse_HID_Interface);
		USB_USBTask();
	}
}

void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	//Joystick_Init();
	//LEDs_Init();
	//Buttons_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Mouse_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	static char counter = 0;
	USB_MouseReport_Data_t* MouseReport = (USB_MouseReport_Data_t*)ReportData;
	int joyX = ReadADC(JOY_X_PIN);
    // Read analog values from joystick Y
    int joyY = ReadADC(JOY_Y_PIN);

	int dx = 0;
	int dy = 0;

    if (joyX < 411){
		dx = calc(joyX);
	} else if (joyX > 611)
	{
		dx = calc(joyX);
	} else {
		dx=0;
	}
	
	if (joyY < 411){
		dy = calc(joyY);
	} else if (joyY > 611)
	{
		dy = calc(joyY);
	} else {
		dy = 0;
	}	



	MouseReport->X += dx;
	MouseReport->Y += dy;

	if(PIND & (1 << LEFT_BUTTON)){
		MouseReport->Button |= (1 << 0);
	}
	if(PIND & (1 << RIGHT_BUTTON)){
		MouseReport->Button |= (1 << 1);
	}
	
	*ReportSize = sizeof(USB_MouseReport_Data_t);
	return true;
}


void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

