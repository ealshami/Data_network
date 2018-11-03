#include "main.h"
#include "Time_Delays.h"
#include "Clk_Config.h"

#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

#include "stm32f4xx_ll_crc.h"

/*
This program flashes an LED to test the nucleo board, while including the header files and '#defines' that
you will need for the later parts of the lab. Use this project as a template for the remaining tasks of the lab.
*/

//Temperature Sensor I2C Address
#define TEMPADR 0x90

//EEPROM I2C Address
#define EEPROMADR 0xA0
		
		
void write_eeprom (int temps);	
int read_eeprom (void);

struct pload {
   int8_t MACdst[6];
   int8_t MACsrc[6];
   uint16_t length;
   int8_t pload[46];
} pload;  

int main(void){
	//Init
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	
	//Configure LED: GPIO Port B, pin 4
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
	LL_GPIO_SetPinSpeed (GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	
	//Configure joystick
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB); // enable peripheral clock
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); // set B5 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); //set B5 as NO pull
	
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOC); // enable peripheral clock
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); // set C0 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); //set C0 as NO pul
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); // set C1 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO); //set C1 as NO pul
	
	
	//Configure LCD
	Configure_LCD_Pins ();
	Configure_SPI1 ();
	Activate_SPI1 ();
	Clear_Screen ();
	Initialise_LCD_Controller ();
	set_font ((unsigned char*) Arial_12);
	
	
	// configure SCL as Alternate function, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);


	// configure SDA as: Alternate, Open Drain, Pull Up:
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetAFPin_8_15 (GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
	LL_GPIO_SetPinOutputType (GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
	
	// Enable the I2C1 Peripheral:
	LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_I2C1);
	LL_I2C_Disable (I2C1); // disable I2C1 prior to configuration
	LL_I2C_SetMode (I2C1, LL_I2C_MODE_I2C);
	LL_I2C_ConfigSpeed (I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2); // set speed to 100 kHz
	LL_I2C_Enable (I2C1); // re-enable I2C1
	
	
	
	for (int i = 0;i<6;i++){
	pload.MACdst[i] = 10; //Sets the MACdst to 0xaaaaaaaaaaaa
	pload.MACsrc[i] = 11; //Sets the MACsrc to 0xbbbbbbbbbbbb
	
	}
	
	pload.length = 46;
	
	
	
	
	int js_centre = 0;
	int js_right = 0;
	int js_left = 0;
	int js_up = 0;
	int js_down = 0;
	char outputString [18]; // buffer to store text for the LCD
	uint16_t temp;
	int eeprom_temp;
	int test_wr;
	
	while(1){
		js_centre = joystick_centre();
		js_right = joystick_right();
		js_left = joystick_left();
		if (js_centre == 1) {
			Clear_Screen();
			temp = readtemp();
			put_string (0,0, "Temp (C*):");
			sprintf (outputString, "%f", temp*0.125); // print to the LCD
			put_string (0,10,outputString);
		}
		if (js_right == 1) {
			Clear_Screen();
			writetemp_eeprom(temp);
			sprintf (outputString, "Written         "); // print to the LCD
			put_string (0,0,outputString);
		
		}
		if (js_left == 1){
			Clear_Screen();
			eeprom_temp = readtemp_eeprom();
		sprintf (outputString, "ETmp: %f", eeprom_temp*0.125); // print to the LCD
		put_string (0,0,outputString);
		}
	}
}

int joystick_centre (void) {
// returns 1 if the joystick is pressed in the centre, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOB, LL_GPIO_PIN_5));
}

int joystick_left (void) {
// returns 1 if the joystick is pressed in the left, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_1));
}

int joystick_right (void) {
// returns 1 if the joystick is pressed in the right, or 0 otherwise
return (LL_GPIO_IsInputPinSet (GPIOC, LL_GPIO_PIN_0));
}




int read_temp (void){

uint16_t temperature = 0; // to store the temperature value
uint16_t hightemp = 0;
uint16_t lowtemp = 0;
LL_I2C_GenerateStartCondition (I2C1); // Start
while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, TEMPADR); // Address + Write
while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
LL_I2C_TransmitData8 (I2C1, 0x00); // set pointer register to the temperature register
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete

LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, TEMPADR+1); // Address + Read
	
	///
while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
hightemp = LL_I2C_ReceiveData8 (I2C1); // read temperature High Byte from
 // Receive Data register.
temperature = hightemp << 8;
pload.pload[1] = hightemp;
LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); // NACK incoming data
while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
lowtemp = LL_I2C_ReceiveData8 (I2C1); // read temperature Low Byte
temperature += lowtemp;
pload.pload[0] = lowtemp;
LL_I2C_GenerateStopCondition (I2C1); // Stop
temperature = temperature >> 5; // bit-shift temperature right, since it's st
return (temperature);


}

void write_eeprom (int temps){

	int temphigh;
	int templow;
LL_I2C_GenerateStartCondition (I2C1); // Start
while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, 0xA0); // Address + Write
while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
LL_I2C_TransmitData8 (I2C1, 0x00); // set pointer register to the internal address
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, 0x00); // set pointer register to the internal address
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete

	
	
	
temphigh = temps << 8;
templow = temps & 0xFF;
	
LL_I2C_TransmitData8 (I2C1, temphigh); // set pointer register to the temperature register
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, templow); // set pointer register to the temperature register
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
LL_mDelay(5000);
LL_I2C_GenerateStopCondition (I2C1); // Stop
}

int read_eeprom (void){

	uint16_t temperature = 0; // to store the temperature value
LL_I2C_GenerateStartCondition (I2C1); // Start
while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, 0xA0); // Address + Write
while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag
	
LL_I2C_TransmitData8 (I2C1, 0x00); // set pointer register to the temperature register
while (!LL_I2C_IsActiveFlag_TXE (I2C1)); // wait until complete
LL_I2C_TransmitData8 (I2C1, 0x00); // Address + Write	
while (!LL_I2C_IsActiveFlag_TXE (I2C1));	


LL_I2C_GenerateStartCondition (I2C1); // Restart (when a read command follows a write)
while (!LL_I2C_IsActiveFlag_SB (I2C1)); // wait until complete

LL_I2C_TransmitData8 (I2C1, 0xA1); // Address + Write
while (!LL_I2C_IsActiveFlag_ADDR (I2C1)); // wait until complete
LL_I2C_ClearFlag_ADDR (I2C1); // clear the ADDR flag	
	
LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_ACK); // ACK incoming data
while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
temperature = LL_I2C_ReceiveData8 (I2C1); // read temperature High Byte from
 // Receive Data register.
temperature = temperature << 8;
LL_I2C_AcknowledgeNextData (I2C1, LL_I2C_NACK); // NACK incoming data
while (!LL_I2C_IsActiveFlag_RXNE (I2C1)); // wait until complete
temperature += LL_I2C_ReceiveData8 (I2C1); // read temperature Low Byte
return (temperature);
	
	




}


