#include "MY_DHT11.h"
#include "dwt_stm32_delay.h"

#define DHT11_PORT GPIOD
#define DHT11_PIN GPIO_PIN_8

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;
int temp_low, temp_high, rh_low, rh_high;
//char temp_char1[2], temp_char2, rh_char1[2], rh_char2;
uint8_t check = 0, result =0;
uint8_t count_DHT=0,vat;

/*
static GPIO_TypeDef* DHT11_PORT;
static uint16_t DHT11_PIN;
*/

//extern GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;

/*
void DHT11_Init(GPIO_TypeDef* DataPort, uint16_t DataPin)
{
	DHT11_PORT = DataPort;
	DHT11_PIN = DataPin;	
}
*/

void DHT11_set_gpio_output (void)
{
	/*Configure GPIO pin output: PA2 */
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
  GPIO_InitStruct.Pin = DHT11_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_start (void)
{
	
	DHT11_set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	DWT_Delay_us(20000);   // wait for 18ms
	DHT11_set_gpio_input ();   // set as input
	DHT11_check_response();
}

void DHT11_check_response (void)
{
	
	DWT_Delay_us (40);
//	vat =HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		DWT_Delay_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) check = 1;
		else check = 0;
	} else {
	
	DHT11_start();
	
	}
	//count_DHT++;
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

}

uint8_t DHT11_read_data (void)
{
	
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		DWT_Delay_us (40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) == 0)   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}



char DHT11_GetTemp_Humidity(uint8_t *Temp, uint8_t *Humidity){

	 
		DHT11_start ();
	  //DHT11_check_response ();
	  Rh_byte1 = DHT11_read_data ();
	  Rh_byte2 = DHT11_read_data ();
	  Temp_byte1 = DHT11_read_data ();
	  Temp_byte2 = DHT11_read_data ();
	  sum = DHT11_read_data();
		
	 if (sum == (Rh_byte1+Rh_byte2+Temp_byte1+Temp_byte2))    
		 // if the data is correct
	  {
			result = 1;
			*Temp = Temp_byte1;
			*Humidity = Rh_byte1;
		}	else {
			result = 0;
		  //printf("\n\r Checksum not matched..Incorrect data from DHT11\n\r");
	  }
		/*
		for(int k =0; k<100;k++){
		DWT_Delay_us(1000);
		}*/
		//count_DHT++;
	return result;

}