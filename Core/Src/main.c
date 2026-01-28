/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stdio.h"
#include "MPU6050.h"

#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"

#include "savedMazes.h"

#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_CS_PORT   GPIOB
#define OLED_CS_PIN    GPIO_PIN_12

#define OLED_DC_PORT   GPIOB
#define OLED_DC_PIN    GPIO_PIN_0

#define OLED_RST_PORT  GPIOA
#define OLED_RST_PIN   GPIO_PIN_4


#define ROWS 10
#define COLS 10


// SPI handle
//extern SPI_HandleTypeDef hspi2;


void loop() {
	// Blue button pressed - repeast the test
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {

		ssd1306_TestAll();

    }

	HAL_Delay(150);
}


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MPU_ADDR (0x68 << 1)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

void print_float(float x) {
    int int_part = (int)x;
    int frac_part = (int)((x - int_part) * 100); // 2 miejsca po przecinku
    printf("%d.%02d\n", int_part, frac_part);
}

typedef enum {
	STATE_HOME,
    STATE_MENU,
    STATE_GAME,
	STATE_WIN
} AppState;

AppState currentState = STATE_HOME;


#define MAZE_W 128
#define MAZE_H 64
#define MAZE_SIZE (MAZE_W * MAZE_H)

uint8_t maze[MAZE_H][MAZE_W];
uint8_t rx_byte;
uint32_t index = 0;
uint8_t receiving = 0;

bool isStart = false;
int whichMap = 0;
int xPos = 1;
int yPos = 1;
int finishPosX = 0;
int finishPosY = 0;

uint32_t textStartTime = 0;
uint8_t textShown = 0;



int move_y(int device_zero, int y){
	if(y>device_zero+10 && yPos<62){
		if(maze[yPos+2][xPos]!=1 && maze[yPos+2][xPos+1]!=1 && maze[yPos+2][xPos-1]!=1)
			yPos+=1;
		printf("idzie w dol");
	}
	else if(y<device_zero-10 && yPos>0){
		if(maze[yPos-2][xPos]!=1 && maze[yPos-2][xPos+1]!=1 && maze[yPos-2][xPos-1]!=1)
			yPos-=1;
		printf("idzie do gory");
	}

	if(y>device_zero+30 && yPos<62){
		if(maze[yPos+2][xPos]!=1 && maze[yPos+2][xPos+1]!=1 && maze[yPos+2][xPos-1]!=1)
			yPos+=1;
	}
	else if(y<device_zero-30 && yPos>0){
		if(maze[yPos-2][xPos]!=1 && maze[yPos-2][xPos+1]!=1 && maze[yPos-2][xPos-1]!=1)
			yPos-=1;
	}

	//optional 3times speed
//	if(y>device_zero+50 && yPos<50){
//		yPos+=1;
//	}
//	else if(y<device_zero-50 && yPos>0){
//		yPos-=1;
//	}
	return yPos;
}

int move_x(int device_zero, int x){
	if(x>device_zero+10 && xPos<126){
		if(maze[yPos][xPos+2]!=1 && maze[yPos+1][xPos+2]!=1 && maze[yPos-1][xPos+2]!=1)
			xPos+=1;
		//printf("idzie w lewo");
	}
	else if(x<device_zero-10 && xPos>0){
		if(maze[yPos][xPos-2]!=1 && maze[yPos+1][xPos-2]!=1 && maze[yPos-1][xPos-2]!=1)
			xPos-=1;
		//printf("idzie w prawo");
	}

	if(x>device_zero+30 && xPos<126){
		if(maze[yPos][xPos+2]!=1 && maze[yPos+1][xPos+2]!=1 && maze[yPos-1][xPos+2]!=1)
			xPos+=1;
	}
	else if(x<device_zero-30 && xPos>0){
		if(maze[yPos][xPos-2]!=1 && maze[yPos+1][xPos-2]!=1 && maze[yPos-1][xPos-2]!=1)
			xPos-=1;
	}

	//optional 3times speed
//	if(x>device_zero+50 && xPos<100){
//		xPos+=1;
//	}
//	else if(x<device_zero-50 && xPos>0){
//		xPos-=1;
//	}
	return xPos;
}

void drawStart(int x, int y){
	ssd1306_DrawPixel(x-1, y, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y-1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x+1, y, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y+1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x+2, y-1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x+2, y+1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-2, y-1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-2, y+1, (SSD1306_COLOR) White);
}

void drawFinish(int x, int y){
	ssd1306_DrawPixel(x, y+1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-1, y+1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x+1, y+1, (SSD1306_COLOR) White);

	ssd1306_DrawPixel(x, y, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y-1, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y-2, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y-3, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x, y-4, (SSD1306_COLOR) White);

	ssd1306_DrawPixel(x-1, y-3, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-1, y-4, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-2, y-3, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-2, y-4, (SSD1306_COLOR) White);

	ssd1306_DrawPixel(x-3, y-3, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-3, y-2, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-4, y-3, (SSD1306_COLOR) White);
	ssd1306_DrawPixel(x-4, y-2, (SSD1306_COLOR) White);
}

void HomeLoop(){
	if((xPos > 15 && xPos < 22) && (yPos > 48 && yPos <= 50)){
		ssd1306_Fill(Black);
		currentState = STATE_MENU;

	}
}

void GameLoop(){

	if(whichMap == 1){
		for(int i=0; i<MAZE_H; i++){
				for(int j=0; j<MAZE_W; j++){
					maze[i][j] = savedMaze1[i][j];
				}
			}
	}else {
		for(int i=0; i<MAZE_H; i++){
						for(int j=0; j<MAZE_W; j++){
							maze[i][j] = savedMaze2[i][j];
						}
					}
	}

	drawMaze(maze);
}

void findFinishXY(uint8_t m[MAZE_H][MAZE_W]){
	for (int i = 0; i < MAZE_H; i++) {
	        for (int j=0; j < MAZE_W; j++) {
	            if (m[i][j] == 3) {
	                finishPosY = i;
	                finishPosX = j;
	                break;
	            }
	        }
	    }
}

void MenuLoop(){

	for(int i=0; i<MAZE_H; i++){
		for(int j=0; j<MAZE_W; j++){
			maze[i][j] = labirynthChoiceMaze[i][j];
		}
	}

	drawMaze(maze);

	if((xPos >= 35 && xPos < 38) && yPos > 35){
		whichMap = 1;
	}else if((xPos >= 69 && xPos < 73) && yPos > 35){
		whichMap = 2;
	}

	printf("wybrano mape! %d\n", whichMap);


	if(whichMap > 0){
		ssd1306_Fill(Black);
		if(whichMap == 1){
				for(int i=0; i<MAZE_H; i++){
						for(int j=0; j<MAZE_W; j++){
							maze[i][j] = savedMaze1[i][j];
						}
					}
			}else {
				for(int i=0; i<MAZE_H; i++){
								for(int j=0; j<MAZE_W; j++){
									maze[i][j] = savedMaze2[i][j];
								}
							}
			}
		findFinishXY(maze);
		drawMaze(maze);
		currentState = STATE_GAME;
	}

}



void drawMaze(uint8_t m[MAZE_H][MAZE_W]){
	for(int i=0; i<MAZE_H; i++){
			for(int j=0; j<MAZE_W; j++){
								  if(m[i][j] == 1)
									  ssd1306_DrawPixel(j, i, (SSD1306_COLOR) White);
								  else if(m[i][j] == 2){
									  //drawStart(j,i);
									  xPos=j;
									  yPos=i;
								  }
								  else if(m[i][j] == 3)
									  drawFinish(j,i);
						  	}
						  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (!receiving)
        {
            if (rx_byte == 0xAA)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
                receiving = 1;
                index = 0;
            }
        }
        else
        {
            if (rx_byte == 0x55)
            {
                receiving = 0;
                ssd1306_Fill(Black);
                for(int i=0; i<MAZE_H; i++){
                	for(int j=0; j<MAZE_W; j++){
              		  if(maze[i][j] == 1)
              			  ssd1306_DrawPixel(j, i, (SSD1306_COLOR) White);
              		  else if(maze[i][j] == 2){
              			  //drawStart(j,i);
              			  xPos=j;
              			  yPos=i;
              		  }
              		  else if(maze[i][j] == 3){
              			drawFinish(j,i);
              		  }

                	}
                }

                ssd1306_UpdateScreen();
            }
            else if (index < MAZE_SIZE)
            {
                maze[index / MAZE_W][index % MAZE_W] = rx_byte;
                index++;
            }
        }

        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

int maze_number = 1;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
	  if(maze_number == 1){
		  for(int i=0; i<MAZE_H; i++){
			for(int j=0; j<MAZE_W; j++){
				maze[i][j] = savedMaze2[i][j];
			}
		  }
		  maze_number = 2;
	  }
	  else if(maze_number == 2){
		  for(int i=0; i<MAZE_H; i++){
			for(int j=0; j<MAZE_W; j++){
				maze[i][j] = savedMaze1[i][j];
			}
		  }
		  maze_number = 1;
	  }



	  ssd1306_Fill(Black);
	  for(int i=0; i<MAZE_H; i++){
	  	for(int j=0; j<MAZE_W; j++){
			  if(maze[i][j] == 1)
				  ssd1306_DrawPixel(j, i, (SSD1306_COLOR) White);
			  else if(maze[i][j] == 2){
				  //drawStart(j,i);
				  xPos=j;
				  yPos=i;
			  }
			  else if(maze[i][j] == 3)
				  drawFinish(j,i);
	  	}
	  }

	  ssd1306_UpdateScreen();

  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  ssd1306_Init();


  for(int i=0; i<MAZE_H; i++){
  	for(int j=0; j<MAZE_W; j++){
		  maze[i][j] = menuMaze[i][j];
  	}
  }
  ssd1306_Fill(Black);
  for(int i=0; i<MAZE_H; i++){
  	for(int j=0; j<MAZE_W; j++){
		  if(maze[i][j] == 1)
			  ssd1306_DrawPixel(j, i, (SSD1306_COLOR) White);
		  else if(maze[i][j] == 2){
			  //drawStart(j,i);
			  xPos=j;
			  yPos=i;
		  }
		  else if(maze[i][j] == 3)
			  drawFinish(j,i);
  	}
  }

  ssd1306_UpdateScreen();




  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x68 << 1, 3, 100) == HAL_OK) {
      printf("MPU6050 znaleziony!\n");
  } else {
      printf("Nie znaleziono MPU6050!\n");
  }
  uint8_t who_am_i = 0;
  uint8_t data = 0x00;
  HAL_I2C_Mem_Read(&hi2c1, 0x68<<1, 0x75, 1, &who_am_i, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Write(&hi2c1, 0x68<<1, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
  HAL_Delay(100);
  printf("Who Am I: 0x%X\n", who_am_i);

	char buf[4];
	MPU6050_init();

	float x;
	float y;
	float z;

	MPU6050_Read_Accel(&x, &y, &z);

	//print_float(x);

	int device_zero_x = (int)x;
	int device_zero_y = (int)y;

//	void startGame(int whichMaze){
//		 //saving data from gyro to x,y,z variables
//		MPU6050_Read_Accel(&x, &y, &z);
//
//		//clearing previous position
//		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) Black);
//		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) Black);
//		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) Black);
//		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) Black);
//
//		//x on gyro is y on screen
//		//changing player position
//		xPos = move_x(device_zero_y ,y);
//		yPos = move_y(device_zero_x, x);
//
//
//
//
//		//displaying new position
//		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) White);
//		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) White);
//		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) White);
//		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) White);
//
//		ssd1306_UpdateScreen();
//		HAL_Delay(100);
//	}
	//gyroLoop(hi2c1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf("pozycja mety: %d\n", currentState);
	  	if(currentState == STATE_GAME){

	  		MPU6050_Read_Accel(&x, &y, &z);

	  		//clearing previous position
	  		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) Black);

	  		xPos = move_x(device_zero_y ,y);
	  		yPos = move_y(device_zero_x, x);
	  		printf("pozycja mety: %d, %d\n", finishPosY, finishPosX);

	  		if((yPos > finishPosY - 4 && yPos < finishPosY + 2) && (xPos > finishPosX - 4 && xPos < finishPosX + 2)){
	  			ssd1306_Fill(Black);
	  			currentState = STATE_WIN;






//	  			ssd1306_UpdateScreen();
	  		}

	  		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) White);

	  		ssd1306_UpdateScreen();
	  		HAL_Delay(100);
	  	}else if(currentState == STATE_WIN){
	  		ssd1306_WriteString("WYGRALES BRAWO!!!", Font_7x10, White);

	  		MPU6050_Read_Accel(&x, &y, &z);

	  		//clearing previous position
	  		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) Black);
	  		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) Black);

	  		xPos = move_x(device_zero_y ,y);
	  		yPos = move_y(device_zero_x, x);

	  		if(yPos > 58){
	  			currentState = STATE_MENU;
	  		}

	  		ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) White);
	  		ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) White);

	  		ssd1306_UpdateScreen();
	  		HAL_Delay(100);

	  	}
	  	else{
	  		 //saving data from gyro to x,y,z variables

	  				MPU6050_Read_Accel(&x, &y, &z);

	  				//clearing previous position
	  				ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) Black);
	  				ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) Black);
	  				ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) Black);
	  				ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) Black);

	  				//x on gyro is y on screen
	  				//changing player position
	  				xPos = move_x(device_zero_y ,y);
	  				yPos = move_y(device_zero_x, x);

	  				ssd1306_DrawPixel(xPos-1, yPos, (SSD1306_COLOR) White);
	  								ssd1306_DrawPixel(xPos, yPos-1, (SSD1306_COLOR) White);
	  								ssd1306_DrawPixel(xPos+1, yPos, (SSD1306_COLOR) White);
	  								ssd1306_DrawPixel(xPos, yPos+1, (SSD1306_COLOR) White);
	  				switch(currentState){
	  					case STATE_HOME:
	  						HomeLoop();
	  						break;
	  					case STATE_MENU:
	  						MenuLoop();
	  						break;
	  				}


	  						ssd1306_UpdateScreen();
	  						HAL_Delay(100);
	  	}

  }


		//displaying new position


		//ssd1306_SetCursor(xPos,yPos);
		//ssd1306_WriteString("napis", Font_7x10, White);
		//char c = 'A';
		//HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = GPIO_PIN_12; // CS
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;  // DC
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;  // RST
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
