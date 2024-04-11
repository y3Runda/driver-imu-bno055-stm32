## Usage

BNO055 Datasheet: https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf

Include `bno055_stm32.h` and `bno055.h` . Pass i2c handler to bno055_assignI2C function and set work mode:

    bno055_assignI2C(&hi2c1);
    bno055_setup();
    bno055_setOperationModeNDOF();

Use bno055_getVectorEuler to receive euler angle data:

    bno055_vector_t v = bno055_getVectorEuler();
    printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);

Use bno055_getVectorQuaternion to receive quaternion data:

    bno055_vector_t v = bno055_getVectorQuaternion();
    printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);

To remap axis, use bno055_setAxisMap in config mode (refer to datasheet page 24):

    bno055_axis_map_t axis = {
	    .x = BNO055_AXIS_X,
	    .x_sign = BNO055_AXIS_SIGN_POSITIVE,
	    .y = BNO055_AXIS_Y,
	    .y_sign = BNO055_AXIS_SIGN_POSITIVE,
	    .z = BNO055_AXIS_Z,
	    .z_sign = BNO055_AXIS_SIGN_POSITIVE
    };
    bno055_setAxisMap(axis);

Full example:

    # main.c
    
    /* USER CODE BEGIN Header */
    /* USER CODE END Header */
    
    /* Includes ------------------------------------------------------------------*/
    #include "main.h"
    #include "i2c.h"
    #include "gpio.h"
    
    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */
    #include "bno055_stm32.h"
    /* USER CODE END Includes */
    
    /* Private typedef -----------------------------------------------------------*/
    /* USER CODE BEGIN PTD */
    
    /* USER CODE END PTD */
    
    /* Private define ------------------------------------------------------------*/
    /* USER CODE BEGIN PD */
    
    /* USER CODE END PD */
    
    /* Private macro -------------------------------------------------------------*/
    /* USER CODE BEGIN PM */
    
    /* USER CODE END PM */
    
    /* Private variables ---------------------------------------------------------*/
    
    /* USER CODE BEGIN PV */
    
    /* USER CODE END PV */
    
    /* Private function prototypes -----------------------------------------------*/
    void SystemClock_Config(void);
    /* USER CODE BEGIN PFP */
    
    /* USER CODE END PFP */
    
    /* Private user code ---------------------------------------------------------*/
    /* USER CODE BEGIN 0 */
    
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
      MX_I2C1_Init();
      /* USER CODE BEGIN 2 */
      bno055_assignI2C(&hi2c1);
      bno055_setup();
      bno055_setOperationModeNDOF();
      /* USER CODE END 2 */
    
      /* Infinite loop */
      /* USER CODE BEGIN WHILE */
      while (1)
      {
        /* USER CODE END WHILE */
        bno055_vector_t v = bno055_getVectorEuler();
        printf("Heading: %.2f Roll: %.2f Pitch: %.2f\r\n", v.x, v.y, v.z);
        v = bno055_getVectorQuaternion();
        printf("W: %.2f X: %.2f Y: %.2f Z: %.2f\r\n", v.w, v.x, v.y, v.z);
        HAL_Delay(1000);
        /* USER CODE BEGIN 3 */
      }
      /* USER CODE END 3 */
    }
    
    /**
      * @brief System Clock Configuration
      * @retval None
      */
    void SystemClock_Config(void)
    {
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
      /** Initializes the CPU, AHB and APB busses clocks 
      */
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
      RCC_OscInitStruct.HSIState = RCC_HSI_ON;
      RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
      RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
      if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
      {
        Error_Handler();
      }
      /** Initializes the CPU, AHB and APB busses clocks 
      */
      RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
      RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
      RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
      RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
      RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
      if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
      {
        Error_Handler();
      }
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
    
      /* USER CODE END Error_Handler_Debug */
    }
    
    #ifdef  USE_FULL_ASSERT
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
         tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
      /* USER CODE END 6 */
    }
    #endif /* USE_FULL_ASSERT */

