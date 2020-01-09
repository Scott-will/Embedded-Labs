/******************************************************************************
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
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

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

char lcd_buffer[6];    // LCD display buffer

uint16_t Tim2_PrescalerValue, Tim3_PrescalerValue,Tim4_PrescalerValue;
TIM_HandleTypeDef    Tim2_Handle, Tim3_Handle,Tim4_Handle;


__IO HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 
uint16_t EE_status=0;
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777}; // the emulated EEPROM can save 3 varibles, at these three addresses.
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type
uint16_t count = 2; //count
char* time[5];
uint16_t BestResult; //best time
uint32_t randtime = 1; //random time
int statevar = 0; //state
uint16_t user_time1; //time for first button press
uint16_t user_time2; //time for second button press
uint16_t EECHECK; //variable check if eeprom is empty
RNG_HandleTypeDef Rng_Handle;
uint16_t system_time = 0; //timer for system
__IO uint16_t Tim3_CCR;




/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
void TIM3_Config(void);
void TIM2_Config(void);

void Button_Push(void); //function to be called when button is pushed

/* Private functions ---------------------------------------------------------*/

int main(void)
{
	HAL_Init();

  SystemClock_Config();
	TIM2_Config();
	TIM3_Config();
	
	Tim3_CCR = 1000; //changed the ccr and prescaler to make interrupt fire every 1ms
	HAL_InitTick(0x0000); //set the systick interrupt priority to the highest

	BSP_LED_Init(LED4);
	BSP_LED_Init(LED5);
	BSP_LED_Off(LED4);

	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);

	//BSP_LCD_GLASS_ScrollSentence((uint8_t*) "  mt3ta4 lab2 starter", 1, 200);
	BSP_LCD_GLASS_DisplayString((uint8_t*)"MT3TA4");	
	
	
//******************* use emulated EEPROM ====================================
	//First, Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	EE_status=EE_Init();
	if(EE_status != HAL_OK)
  {
		Error_Handler();
  }
	

	

// then can write to or read from the emulated EEPROM
	
//*********************use RNG ================================  
Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	Hal_status=HAL_RNG_Init(&Rng_Handle);   //go to msp.c to see further low level initiation.
	
	if( Hal_status != HAL_OK)
  {
    Error_Handler();
  }
//then can use RNG

	

  /* Infinite loop */
  while (1)
  {
		

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

                                            

  // MSI is enabled after System reset at 4Mhz, PLL not used 
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6; // RCC_MSIRANGE_6 is for 4Mhz. _7 is for 8 Mhz, _9 is for 16..., _10 is for 24 Mhz, _11 for 48Hhz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40; 
  RCC_OscInitStruct.PLL.PLLR = 2;  //2,4,6 or 8
  RCC_OscInitStruct.PLL.PLLP = 7;   // or 17.
  RCC_OscInitStruct.PLL.PLLQ = 4;   //2, 4,6, 0r 8  ===the clock for RNG will be 4Mhz *N /M/Q =40Mhz. which is nearly 48
	
	
	
	
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    // Initialization Error 
    while(1);
  }

  // Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers 
  // Set 0 Wait State flash latency for 4Mhz 
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  // Disable Power Control clock 
  __HAL_RCC_PWR_CLK_DISABLE();
}
//after RCC configuration, for timmer 2---7, which are one APB1, the TIMxCLK from RCC is 4MHz

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    Tim3 is of 16 bits. Timer 2..7 is on APB1.
	
		Since the clock source is MSI, and the clock range is RCC_MSIRANGE_6, SystemCoreClock=4Mhz.
	
		Since RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1, HCLK=4Mhz.
	
		Since RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1, PCLK1=4MHz, and TIM3CLK=4Mhz.
		(if the APB1 prescaler is not 1, the timer clock frequency will be 2 times of APB1 frequency)
	
		 that is, for current RCC config, the the Timer3 frequency=SystemCoreClock.
	
		To get TIM3's counter clock at 10 KHz, for example, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	
		i.e: Prescaler = (SystemCoreClock /10 KHz) - 1
       
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint16_t) (SystemCoreClock/ 1000000) - 1; //fire interrupt every 1ms
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
 
  Tim3_Handle.Init.Period = 1000 - 1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}

void TIM2_Config(void){
	Tim2_PrescalerValue = (uint16_t) (SystemCoreClock/ 1) - 1; //1hz interrupt
	
	Tim2_Handle.Instance = TIM2;
	
	Tim2_Handle.Init.Period = 2000;
  Tim2_Handle.Init.Prescaler = Tim2_PrescalerValue;
  Tim2_Handle.Init.ClockDivision = 0;
  Tim2_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim2_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim2_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}







void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
			case GPIO_PIN_0: 		 //SEL_JOY_PIN    			
								statevar ++; //increment to next state
								Button_Push(); //call function to run game
                
			
								break;	
			case GPIO_PIN_1: //LEFT_JOY_PIN  
							statevar ++;
							Button_Push();

							break;
			case GPIO_PIN_2: //RIGHT_JOY_PIN  
							statevar ++;
							Button_Push();
			
							break;
			case GPIO_PIN_3:  //UP_JOY_PIN 
							statevar ++;
							Button_Push();

							break;
			
			case GPIO_PIN_5:  //DOWN_JOY_PIN
						statevar ++;
						Button_Push();

							break;
			default://
						//default
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*)"OTHER");
	  } 
}



//write logic code here
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
	
	if ((*htim).Instance==TIM2 && statevar == 0){    
		BSP_LED_Toggle(LED5); //BLINK LED AT START 
	}
		
	
	if((*htim).Instance == TIM3 && statevar >= 1){ //TIMING HOW LONG IT TAKES USER TO PUSH BUTTON
	system_time++; //increment timer every 1 ms if button is pushed first time
	}

  if(system_time == randtime){
    BSP_LED_On(LED5); //TURN ON LED AFTEr RANDTIME
  }
    if(system_time >= randtime && statevar == 1){ //display time between when light turns on and when user presses button
        if((system_time - (user_time1 + randtime))%100 == 0){
            sprintf(lcd_buffer, "%d", (system_time - (user_time1 + randtime)));
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
        }
    } 
	
	
    //Printing best time value to the screen 
    if(statevar == 3 && system_time%1000 == 0){ //FLASHES BEST RESULT 
      sprintf(lcd_buffer, "%d", BestResult); //
      if(count%2 == 0){
        BSP_LCD_GLASS_Clear();
        BSP_LCD_GLASS_DisplayString("RECORD");
      }
      else{
        BSP_LCD_GLASS_Clear();
		    BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
      }
		  count++;
    }

   
      
    
  }

	


//logic end

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim){  
		if ((*htim).Instance==TIM4){
			 BSP_LED_Toggle(LED5); 
	}
	
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h

}

void Button_Push(void){ //function to run game
	if(statevar == 1){ //when they push the button first time, 
                user_time1 = system_time;
								}

                if(statevar == 1){ //AFTER USER PUSH BUTTON, GENERATE RANDOM NUMBER
                    BSP_LED_Off(LED5);
                    HAL_RNG_GenerateRandomNumber(&Rng_Handle, &randtime); //generate random number
                        
                    for (int i = 12; i < 33; i++){ //make number between 0 - 4000
                        randtime &= ~(1<<i);
                    }
                }

                if(statevar == 2){
                  user_time2 = system_time; //after second button press, record user time
		
		
                    if((user_time2 - user_time1) < randtime){ //IF PUSHED BEFORE  light turned on THEN RESTART GAME
                            BSP_LCD_GLASS_Clear();
                            BSP_LCD_GLASS_DisplayString("CHEAT");
                    
														statevar = 0;
                            system_time = 0;
                    }
                

                    //}
                    if((user_time2 - user_time1) > randtime){
                    EE_ReadVariable(VirtAddVarTab[1], &EECHECK); //CHECK TO SEE IF EEprom has been written to
                    if(EECHECK != 1){                             //if not write to eeprom for first time
                    BestResult = (user_time2 - user_time1) - randtime; //store reaction time in best result
                    EE_WriteVariable(VirtAddVarTab[0], BestResult); //write to eeprom
                            sprintf(lcd_buffer, "%d", BestResult);
                            BSP_LCD_GLASS_Clear();
                            BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer); //display time
                    EE_WriteVariable(VirtAddVarTab[1], 1); //set flag to show that the eeprom has been written to
                    statevar = 3;
                }
                    
                    EE_ReadVariable(VirtAddVarTab[0], &BestResult); //READ BEST TIME
                    if(EECHECK == 1){
                        if(((user_time2 - user_time1)- randtime) < BestResult){ //CHECK IF TIME IS LESS
                            BestResult = (user_time2 - user_time1) - randtime; //STORE reaction TIME IN BEST RESULT
                            EE_WriteVariable(VirtAddVarTab[0], BestResult); //WRITE BEST RESULT TO EEPROM
                        }
                        
                    sprintf(lcd_buffer, "%u", (user_time2 - user_time1)- randtime); //display reaction time
                    BSP_LCD_GLASS_Clear();
                    BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);			
                    statevar = 3; //INCREMENT TO NEXT STATE 
                    }
                }
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