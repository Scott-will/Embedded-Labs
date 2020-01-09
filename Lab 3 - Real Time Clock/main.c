/**

******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    21-April-2017
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32L4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************


this program: 

1. This project needs the libraray file i2c_at2464c.c and its header file. 
2. in the i2c_at2464c.c, the I2C SCL and SDA pins are configured as PULLUP. so do not need to pull up resistors (even do not need the 100 ohm resisters).
NOTE: students can also configure the TimeStamp pin 	

*/




/* Includes ------------------------------------------------------------------*/
#include "main.h" //adding libraries requred for code"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  pI2c_Handle;

RTC_HandleTypeDef RTCHandle;
RTC_DateTypeDef RTC_DateStructure;
RTC_TimeTypeDef RTC_TimeStructure;

__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

//memory location to write to in the device
__IO uint16_t memLocation = 0x000A; //pick any location within range

  

char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  //   
char datestring[6]={0};


//New variables here
//uint16_t Tim2_PrescalerValue;
//TIM_HandleTypeDef Tim2_Handle;


uint8_t wd, dd, mo, yy, ss, mm, hh; // for weekday, day, month, year, second, minute, hour

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed

//__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>800ms)

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

void RTC_Config(void);
void RTC_AlarmAConfig(void);

//our functions
void currentDate(void); //function prototypes
void RTC_SET_TIME(void);
void RTC_SET_DATE(void);
void Store_Time(void);
void Get_Time(void);
void Show_Recent_Time(void);
	

//our variables
int left_push = 0; //variables that incriment when joystick direction is pressed
int right_push = 0;
uint8_t select_push = 0;
bool leftpressed = false;  //Variables to determine if joystick is pressed in certain directions
bool rightpressed = false;
bool screen_on = false; //flashes screen to indicate edit mode
int dateCounter = 0;
int selpressed = 0; //incriments in edit mode when user presses joystick in
bool writing = false; //variable to determine if we are writing

uint8_t Time1[7]; //int array to show last 2 times user pressed joystick
uint8_t Time2[7];

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

uint8_t data1 =0x67,  data2=0x68;
uint8_t readData=0x00;
uint16_t EE_status;

int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4 
       - Low Level Initialization
     */

	
	

	HAL_Init();
	
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
  
	SystemClock_Config();   
											
	
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest, !!!This line need to be after systemClock_config()

	
	BSP_LCD_GLASS_Init();
	
	BSP_JOY_Init(JOY_MODE_EXTI);

	BSP_LCD_GLASS_DisplayString((uint8_t*)"YEET");	
	HAL_Delay(1000);


//configure real-time clock
	RTC_Config();
	
	RTC_AlarmAConfig();
	
	I2C_Init(&pI2c_Handle);
		Get_Time();

//*********************Testing I2C EEPROM------------------


	/*BSP_LCD_GLASS_Clear();
	if ( == readData) {
		
			
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 1 X");
	}	
	
	HAL_Delay(1000);
	
	readData=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+1);

	BSP_LCD_GLASS_Clear();
	if (data2 == readData) {
			BSP_LCD_GLASS_DisplayString((uint8_t*)"r 2 ok");;
	}else{
			BSP_LCD_GLASS_DisplayString((uint8_t *)"r 2 X");
	}	

	HAL_Delay(1000);*/
	


//******************************testing I2C EEPROM*****************************	
		

  /* Infinite loop */
  while (1)
  {
			//the joystick is pulled down. so the default status of the joystick is 0, when pressed, get status of 1. 
			//while the interrupt is configured at the falling edge---the moment the pressing is released, the interrupt is triggered.
			//therefore, the variable "selpressed==1" can not be used to make choice here.
			if (BSP_JOY_GetState() == JOY_SEL) {
					SEL_Pressed_StartTick=HAL_GetTick(); 
					while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
						if ((HAL_GetTick()-SEL_Pressed_StartTick)>800) {	
							currentDate();	//calls the current date function when joystick is held for more than 800ms
						} 
					}
					
			}					
}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = MSI
  *            SYSCLK(Hz)                     = 4000000
  *            HCLK(Hz)                       = 4000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            Flash Latency(WS)              = 0
  * @param  None
  * @retval None
  */



void SystemClock_Config(void)
{ 
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};                                            

  // RTC requires to use HSE (or LSE or LSI, suspect these two are not available)
	//reading from RTC requires the APB clockis 7 times faster than HSE clock, 
	//so turn PLL on and use PLL as clock source to sysclk (so to APB)
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;     //RTC need either HSE, LSE or LSI           
  
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;  
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue= RCC_MSICALIBRATION_DEFAULT;
  
	//RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;//RCC_PLL_NONE;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;   //PLL source: either MSI, or HSI or HSE, but can not make HSE work.
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  
	//the PLL will be MSI (4Mhz)*N /M/R = 

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; //the freq of pllclk is MSI (4Mhz)*N /M/R = 80Mhz 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
	
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)   //???
  {
    // Initialization Error 
    while(1);
  }

  // The voltage scaling allows optimizing the power consumption when the device is
  //   clocked below the maximum system frequency, to update the voltage scaling value
  //   regarding system frequency refer to product datasheet.  

  // Enable Power Control clock 
  __HAL_RCC_PWR_CLK_ENABLE();

  if(HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Disable Power Control clock   //why disable it?
  __HAL_RCC_PWR_CLK_DISABLE();      
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz



void RTC_Config(void) {
	RTC_TimeTypeDef RTC_TimeStructure; //structs to define time and date variables
	RTC_DateTypeDef RTC_DateStructure;
	

	//****1:***** Enable the RTC domain access (enable wirte access to the RTC )
			//1.1: Enable the Power Controller (PWR) APB1 interface clock:
        __HAL_RCC_PWR_CLK_ENABLE();    
			//1.2:  Enable access to RTC domain 
				HAL_PWR_EnableBkUpAccess();    
			//1.3: Select the RTC clock source
				__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);    
				//RCC_RTCCLKSOURCE_LSI is defined in hal_rcc.h
	       // according to P9 of AN3371 Application Note, LSI's accuracy is not suitable for RTC application!!!! 
				
			//1.4: Enable RTC Clock
			__HAL_RCC_RTC_ENABLE();   //enable RTC --see note for the Macro in _hal_rcc.h---using this Marco requires 
																//the above three lines.
			
	
			//1.5  Enable LSI
			__HAL_RCC_LSI_ENABLE();   //need to enable the LSI !!!
																//defined in _rcc.c
			while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY)==RESET) {}    //defind in rcc.c
	
			// for the above steps, please see the CubeHal UM1725, p616, section "Backup Domain Access" 	
				
				
				
	//***2.*****  Configure the RTC Prescaler (Asynchronous and Synchronous) and RTC hour 
        
		
		//************students: need to complete the following lines******************************
		//**************************************************************************************				
				RTCHandle.Instance = RTC;
				RTCHandle.Init.HourFormat = RTC_HOURFORMAT_12; //12 hour display
				
				//variables used for i2c comms
				RTCHandle.Init.AsynchPrediv = 127; //2^7 -1
				RTCHandle.Init.SynchPrediv = 255; //2^8 -1 to get 1hz frequency
				
				
				RTCHandle.Init.OutPut = RTC_OUTPUT_ALARMA; //using alarm a
				RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH; //high polarity
				RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN; //open drain
				
			
				if(HAL_RTC_Init(&RTCHandle) != HAL_OK)
				{
					BSP_LCD_GLASS_Clear(); 
					BSP_LCD_GLASS_DisplayString((uint8_t *)"RT I X"); 	
				}
	//******************************************************************************************/
	
	
	
	//****3.***** init the time and date
				
				
 		//*****************Students: please complete the following lnes*****************************
		///****************************************************************************************		
				RTC_DateStructure.Year = 19; //2019
				RTC_DateStructure.Month = 1; //october
				RTC_DateStructure.Date = 1; // 24th of october
				RTC_DateStructure.WeekDay = RTC_WEEKDAY_THURSDAY;
				
				if(HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better 
															//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"D I X");
				} 
  
  
				RTC_TimeStructure.Hours = 4;  //4:52:00 pm should change to values in eeprom after
				RTC_TimeStructure.Minutes = 52; 
				RTC_TimeStructure.Seconds = 00;
				RTC_TimeStructure.TimeFormat = RTC_HOURFORMAT12_AM;
				RTC_TimeStructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				RTC_TimeStructure.StoreOperation = RTC_STOREOPERATION_RESET;
				
				if(HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure,RTC_FORMAT_BIN) != HAL_OK)   //BIN format is better
																																					//before, must set in BCD format and read in BIN format!!
				{
					BSP_LCD_GLASS_Clear();
					BSP_LCD_GLASS_DisplayString((uint8_t *)"T I X");
				}	
	  
 //********************************************************************************/



				
			__HAL_RTC_TAMPER1_DISABLE(&RTCHandle);
			__HAL_RTC_TAMPER2_DISABLE(&RTCHandle);	
				//Optionally, a tamper event can cause a timestamp to be recorded. ---P802 of RM0090
				//Timestamp on tamper event
				//With TAMPTS set to �1 , any tamper event causes a timestamp to occur. In this case, either
				//the TSF bit or the TSOVF bit are set in RTC_ISR, in the same manner as if a normal
				//timestamp event occurs. The affected tamper flag register (TAMP1F, TAMP2F) is set at the
				//same time that TSF or TSOVF is set. ---P802, about Tamper detection
				//-------that is why need to disable this two tamper interrupts. Before disable these two, when program start, there is always a timestamp interrupt.
				//----also, these two disable function can not be put in the TSConfig().---put there will make  the program freezed when start. the possible reason is
				//-----one the RTC is configured, changing the control register again need to lock and unlock RTC and disable write protection.---See Alarm disable/Enable 
				//---function.
				
			HAL_RTC_WaitForSynchro(&RTCHandle);	
			//To read the calendar through the shadow registers after Calendar initialization,
			//		calendar update or after wake-up from low power modes the software must first clear
			//the RSF flag. The software must then wait until it is set again before reading the
			//calendar, which means that the calendar registers have been correctly copied into the
			//RTC_TR and RTC_DR shadow registers.The HAL_RTC_WaitForSynchro() function
			//implements the above software sequence (RSF clear and RSF check).	
}


void RTC_AlarmAConfig(void)
{
	RTC_AlarmTypeDef RTC_Alarm_Structure;

	//**************students:  you need to set the followint two lines****************
	//********************************************************************************
	
	RTC_Alarm_Structure.Alarm = RTC_ALARM_A; //using alarm a
  	RTC_Alarm_Structure.AlarmMask = RTC_ALARMMASK_ALL; //update for every value
	//RTC_ALARMSUBSECONDMASK_ALL;
	
	
	//********************************************************************************/			
  
  if(HAL_RTC_SetAlarm_IT(&RTCHandle,&RTC_Alarm_Structure,RTC_FORMAT_BCD) != HAL_OK)
  {
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *)"A S X");
  }

	__HAL_RTC_ALARM_CLEAR_FLAG(&RTCHandle, RTC_FLAG_ALRAF); //without this line, sometimes(SOMETIMES, when first time to use the alarm interrupt)
																			//the interrupt handler will not work!!! 		

		//need to set/enable the NVIC for RTC_Alarm_IRQn!!!!
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);   
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 3, 0);  //not important ,but it is better not use the same prio as the systick
	
}

//You may need to disable and enable the RTC Alarm at some moment in your application
HAL_StatusTypeDef  RTC_AlarmA_IT_Disable(RTC_HandleTypeDef *hrtc) 
{ 
 	// Process Locked  
	__HAL_LOCK(hrtc);
  
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_DISABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_DISABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  
}


HAL_StatusTypeDef  RTC_AlarmA_IT_Enable(RTC_HandleTypeDef *hrtc) 
{	
	// Process Locked  
	__HAL_LOCK(hrtc);	
  hrtc->State = HAL_RTC_STATE_BUSY;
  
  // Disable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);
  
  // __HAL_RTC_ALARMA_ENABLE(hrtc);
    
   // In case of interrupt mode is used, the interrupt source must disabled 
   __HAL_RTC_ALARM_ENABLE_IT(hrtc, RTC_IT_ALRA);


 // Enable the write protection for RTC registers 
  __HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);
  
  hrtc->State = HAL_RTC_STATE_READY; 
  
  // Process Unlocked 
  __HAL_UNLOCK(hrtc);  

}




/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		               //SELECT button					
						if(rightpressed == false){ //if select button is pressed, gets the current values for time and date
							HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
							ss = RTC_TimeStructure.Seconds; //saving each RTC parameter into a variable
							mm = RTC_TimeStructure.Minutes;
							hh = RTC_TimeStructure.Hours;
							wd = RTC_DateStructure.WeekDay;
							dd = RTC_DateStructure.Date;
							mo = RTC_DateStructure.Month;
							yy = RTC_DateStructure.Year;
							 
							Store_Time(); //calls a function to store them to eeprom
						}
						select_push ++;//incimenting variable used in time and date edit mode
						//select_push %= 2; //makes sure that select_push doesnt inciment higher than 1
						break;
						  	
			case GPIO_PIN_1:     //left button						
							if(rightpressed == false){
								leftpressed = !leftpressed; //toggling leftpressed
							}
							if(leftpressed == true && rightpressed == false){ //if left direction is pushed, shows last 2 times
								Get_Time();
								Show_Recent_Time();
							}
							switch(left_push){
								case 1:
									RTC_TimeStructure.Minutes = (RTC_TimeStructure.Minutes + select_push)%60; //updates time variable
									break;
								case 2:
									RTC_TimeStructure.Hours = (RTC_TimeStructure.Hours + select_push)%13; //updates time variable
									break;
								case 3:
									RTC_DateStructure.Date = (RTC_DateStructure.Date + select_push)%31 + 1; //updates date variable
									break;
								case 4:
									RTC_DateStructure.Month = (RTC_DateStructure.Month + select_push)%12 + 1; //updates date variable
									break;
								case 5:
									RTC_DateStructure.Year = (RTC_DateStructure.Year + select_push)%100; //updates date variable
									break;
								case 6:
									RTC_DateStructure.WeekDay = (RTC_DateStructure.WeekDay + select_push)%7 + 1;
									break;
								case 0:
									RTC_TimeStructure.Seconds = (RTC_TimeStructure.Seconds + select_push)%60; //updates time variable
									break;
							}
							HAL_RTC_SetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
							HAL_RTC_SetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
							left_push ++;
							select_push = 0;
							left_push %= 7; 
							break;
			case GPIO_PIN_2:    //right button						  to play again.
							rightpressed = !rightpressed; //toggling rightpressed
							switch(left_push){
								case 1:
									RTC_TimeStructure.Minutes = (RTC_TimeStructure.Minutes + select_push)%60; //updates time variable
									/*BSP_LCD_GLASS_Clear();
									BSP_LCD_GLASS_DisplayString((uint8_t*)("FART"));
									HAL_Delay(500);*/
									break;
								case 2:
									RTC_TimeStructure.Hours = (RTC_TimeStructure.Hours + select_push)%13; //updates time variable
									break;
								case 3:
									RTC_DateStructure.Date = (RTC_DateStructure.Date + select_push)%31 + 1; //updates date variable
									break;
								case 4:
									RTC_DateStructure.Month = (RTC_DateStructure.Month + select_push)%12 + 1; //updates date variable
									break;
								case 5:
									RTC_DateStructure.Year = (RTC_DateStructure.Year + select_push)%100; //updates date variable
									break;
								case 6:
									RTC_DateStructure.WeekDay = (RTC_DateStructure.WeekDay + select_push)%7 + 1;
									break;
								case 0:
									RTC_TimeStructure.Seconds = (RTC_TimeStructure.Seconds + select_push)%60; //updates time variable
									break;
							}
							HAL_RTC_SetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
							HAL_RTC_SetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
							right_push ++;
							right_push %= 3;  //sets right_push value back to 0
							right_push = 0;
							left_push = 0;
							select_push = 0;
							
							if(rightpressed == true){ //if rightpressed is true, calls function to set time
								RTC_SET_TIME();
							}
							break;
			/*case GPIO_PIN_3:    //up button							
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"up");
							break;
			
			case GPIO_PIN_5:    //down button						
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"down");
							break;
			case GPIO_PIN_14:    //down button						
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t*)"PE14");
							break;			
		*/		default://
						//default
						break;
	  } 
}

void RTC_SET_TIME(){
	
	sprintf((char*)lcd_buffer, "%02d%02d%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds); //stores time in lcd_buffer
	BSP_LCD_GLASS_Clear(); // display to lcd
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);

	//select_push = 0;
	if(left_push == 0){ //Flashes seconds to signify we are setting seconds
	
		if(screen_on == false){ 
			sprintf((char*)lcd_buffer, "%02d%02d  ", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			sprintf((char*)lcd_buffer, "%02d%02d%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  (RTC_TimeStructure.Seconds + select_push)%60); //shows the incremented value of seconds by adding number of times user has pushed select push
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
			
		}
	}
	else if(left_push == 1){ //Flashes minutes to signify we are setting minutes
		if(screen_on == false){
			sprintf((char*)lcd_buffer, "%02d  %02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Seconds);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			sprintf((char*)lcd_buffer, "%02d%02d%02d", RTC_TimeStructure.Hours, (RTC_TimeStructure.Minutes + select_push)%60,  RTC_TimeStructure.Seconds);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
	}
	else if(left_push == 2){ //Flashes hours to signify we are setting hours
		if(screen_on == false){
			sprintf((char*)lcd_buffer, "  %02d%02d", RTC_TimeStructure.Minutes, RTC_TimeStructure.Seconds);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			sprintf((char*)lcd_buffer, "%02d%02d%02d", (RTC_TimeStructure.Hours + select_push)%13, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds);
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
	}
	else{
		RTC_SET_DATE();
	}
}

void RTC_SET_DATE(void){

	if(left_push == 3){ //flashes date to change
		if(screen_on == false){
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "DATE%02d", (RTC_DateStructure.Date + select_push)%31 + 1); //modulus 31 to make sure that we dont go over number of days in a month
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "DATE"); //flashes the date value on and off
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
	}

	else if(left_push == 4){ //toggling the month for user edit
		if(screen_on == false){
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "MNTH%02d", (RTC_DateStructure.Month + select_push)%12 + 1); //add one since you cant have month 00
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "MNTH"); //flashes the month value on and off
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
	}

	else if(left_push == 5){ //toggling the year for user edit
		if(screen_on == false){
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "YEAR%02d", (RTC_DateStructure.Year + select_push)%100);
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
		else{
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "YEAR"); //flashes the year value on and off
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		}
	}

	else if(left_push == 6){ //toggling the weekday for user edit
		if(screen_on == false){
			char* day = "Mo";
			switch((RTC_DateStructure.WeekDay + select_push)%7){ //switch case for each day of the week
				case 1:
					day = "Mo";
					break;
				case 2:
					day = "Tu";
					break;
				case 3:
					day = "We";
					break;
				case 4:
					day = "Th";
					break;
				case 5:
					day = "Fr";
					break;
				case 6:
					day = "Sa";
					break;
				case 0:
					day = "Su"; //case 0 is set to sunday for %7 to work successfully
					break;
			}
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "WEEK%02s", day);
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
			 //updates weekday variable
		}
		else{
			BSP_LCD_GLASS_Clear();
			sprintf((char*)lcd_buffer, "WEEK");
			BSP_LCD_GLASS_DisplayString((char*)lcd_buffer); //flashes weekday variable on and off
		}
	}

	//not sure if this has to be here
	// BSP_LCD_GLASS_Clear(); // display to lcd
	// BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
}

void Store_Time(void){
	//write to eeprom
	writing = true; //signify we are writing to eeprom
	for(int x = 0; x < 7; x++){ //copy time1 to time2
		Time2[x] = Time1[x]; 
	}
	Time1[0] = yy; //setting each list indicie to a time variable
	Time1[1] = mo;
	Time1[2] = dd;
	Time1[3] = wd;
	Time1[4] = hh;
	Time1[5] = mm;
	Time1[6] = ss;
	for(int x = 0; x < 7; x++){ //write time 1 to the eeprom
		EE_status = I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation + x, Time1[x]);
		if(EE_status != HAL_OK){
			I2C_Error(&pI2c_Handle);
		}
		
		if (EE_status == HAL_OK){ //let user know that the daa is being written to eeprom
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Writing");
			HAL_Delay(100);
		}
	}
	for(int x = 7; x< 14; x++){ //write time2 to a different memory location than time1
		EE_status = I2C_ByteWrite(&pI2c_Handle, EEPROM_ADDRESS, memLocation + x, Time2[x-7]);
		if(EE_status != HAL_OK){
			I2C_Error(&pI2c_Handle);
		}
		
		if (EE_status == HAL_OK){
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t*)"Writing");
			HAL_Delay(100);
		}
	}
	writing = false; //ends the writing loop
	
}

void Get_Time(void){ //read time form EEPROM
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Reading");
	for(int x = 0; x < 14; x++){
		readData = I2C_ByteRead(&pI2c_Handle, EEPROM_ADDRESS, memLocation+x);
		if(x < 7){ //stores data into time 1
			Time1[x] = readData;
		}
		else{
			Time2[x-7] = readData; //stores data into time 2
		}
	}
}

void Show_Recent_Time(void){ //displays the two times the user pressed the sel button
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Time 1:");
	HAL_Delay(1000);
	BSP_LCD_GLASS_Clear();
	sprintf((char*)lcd_buffer, "%02d%02d%02d", Time1[4], Time1[5], Time1[6]); //displays the hh, mm, ss values from the time array
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
	HAL_Delay(1000);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t*)"Time 2:");
	HAL_Delay(1000);
	BSP_LCD_GLASS_Clear();
	sprintf((char*)lcd_buffer, "%02d%02d%02d", Time2[4], Time2[5], Time2[6]); //displays second most recent time 
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
	HAL_Delay(1000);
	leftpressed = false;
}
//Function that gets called by the interupt
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    BSP_LED_Toggle(LED5); //toggle led

	if(rightpressed == false && writing == false && leftpressed == false){ // shows current time on screen
	HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN); //stores current time in rtc_structure
	sprintf((char*)lcd_buffer, "%02d%02d%02d", RTC_TimeStructure.Hours, RTC_TimeStructure.Minutes,  RTC_TimeStructure.Seconds);// store to lcd_buffer
	BSP_LCD_GLASS_Clear(); //get time and store in string, then display to lcd
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);

	//HAL_Delay(1000);
	HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	}

	if(rightpressed == true){ //toggles value of screen_on for setting time
		 screen_on = !screen_on;
		 
		 RTC_SET_TIME(); //function to set time
	}

	dateCounter ++; 



		
	
//
}

void currentDate(void){ //displays current date
	HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle, &RTC_DateStructure, RTC_FORMAT_BIN);
	if(dateCounter == 0){
	BSP_LCD_GLASS_Clear();
	sprintf((char*)lcd_buffer, "DATE%02d", RTC_DateStructure.Date); //displays the current date value
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
	}
	if(dateCounter == 1){
	BSP_LCD_GLASS_Clear();
	sprintf((char*)lcd_buffer, "MNTH%02d", RTC_DateStructure.Month); //displays the current month value
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
	}
	if(dateCounter == 2){
	BSP_LCD_GLASS_Clear();
	sprintf((char*)lcd_buffer, "YEAR%02d", RTC_DateStructure.Year); //displays the current year value
	BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
	}
	if(dateCounter == 3){
		char* day = "Mo";
		switch(RTC_DateStructure.WeekDay){ //switch case for displaying each day of the week
				case 1:
					day = "Mo";
					break;
				case 2:
					day = "Tu";
					break;
				case 3:
					day = "We";
					break;
				case 4:
					day = "Th";
					break;
				case 5:
					day = "Fr";
					break;
				case 6:
					day = "Sa";
					break;
				case 7:
					day = "Su"; //case 0 is set to sunday for %7 to work successfully
					break;
			}
		BSP_LCD_GLASS_Clear();
		sprintf((char*)lcd_buffer, "WEEK%02s", day);
		BSP_LCD_GLASS_DisplayString((char*)lcd_buffer);
		
	}
	if(dateCounter > 3){
		dateCounter = 0;
		
	}

}

static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {
  }
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
