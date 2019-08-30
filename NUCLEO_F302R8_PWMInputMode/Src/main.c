/* USER CODE BEGIN Header */
/*
 * @ 名称: main.c
 * @ 作者: SummerFalls
 * @ 日期: 2019-8-30
 * @ 版本: V1.0
 * @ 描述:
 * MIT License. Copyright (c) 2019 SummerFalls.
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h> /* 数据类型 */

#include <stdio.h>  /* 输入输出 */
#include <stdlib.h> /* 常用实用函数 */
#include <stddef.h> /* 常用定义 */
#include <string.h> /* 字符串处理 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t TIM15_CC1;
    uint16_t TIM15_CC2;
} TIM_IT_CNT_TypeDef;

typedef struct {
    uint32_t Period;
    uint32_t PulseWidth;
    uint32_t DutyCycle;
    uint32_t Frequency;
    uint32_t CC1_OverflowCnt;
    uint32_t CC2_OverflowCnt;
} PWM_IN_TypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define F_SAMPLING 8000000UL /* PWM输入捕获采样频率_单位: Hz */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TIM_IT_CNT_TypeDef g_TIM_IT_CNT = { 0 };
PWM_IN_TypeDef g_PWM_IN = { 0 };
LL_RCC_ClocksTypeDef RCC_Clocks = { 0 };
uint32_t g_Cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void GTek_PWM_InputMode_Init(void) {
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH2);

    LL_TIM_EnableCounter(TIM15);

//    LL_TIM_DisableIT_UPDATE(TIM15);
    LL_TIM_EnableIT_UPDATE(TIM15);
    LL_TIM_EnableIT_CC1(TIM15);
    LL_TIM_EnableIT_CC2(TIM15);
}

/*******************************************************************************
 * @name   : TIM1_BRK_TIM15_IRQHandler
 * @brief  : TIM1_刹车__TIM15_PWM输入模式
 * @param  : void
 * @retval : void
 * @note   :
 *           PWM 输入模式_TI信号输入引脚为TIM15_CH2
 *           CH1_Indirect_下降沿触发_PWM高电平脉宽
 *           CH2___Direct_上升沿触发_PWM周期
 *
 *           以下请参照参考手册
 *           654页: CKD[1:0]位(Clock Division)
 *           665页：ICxF[3:0]位(Input Capture x Filter)
 *
 *           TIMx_CLK = HCLK / (Prescaler + 1) = 64 MHz / (7 + 1) = 8 MHz
 *           f(DTS) = TIMx_CLK / CKD = 8 MHz / 1 = 8 MHz
 *           因为 ICxF[3:0] = 1，所以：
 *           1. (ICxF[3:0] = 0   ) f(sampling) = f(DTS)
 *              (ICxF[3:0] = 1~3 ) f(sampling) = TIMx_CLK
 *              (ICxF[3:0] = 4~15) f(sampling) = f(DTS) / Y
 *           2. 连续检测周期数 N = 2
 *
 *           采样周期宽度 = 1s / f(sampling) = 0.125 us
 *           连续采样周期宽度 = 采样周期宽度 * N = 0.25 us
 *           所以如果输入电平信号宽度小于 0.25 us 时，会被滤除，不被采样
 *
 *           假设输入的PWM占空比精度为：1%
 *           则必须满足输入的PWM_Period >= 0.25 us * 100 = 25 us
 *           即理论允许采样的最大PWM频率为：40 KHz
 *
 *           PWM_Freq = f(sampling) / g_PWM_IN.Period
 *           PWM_DutyCycle = g_PWM_IN.PulseWidth / g_PWM_IN.Period
 *
 *           测试输入PWM 8 Hz ~ 10 KHz
 *******************************************************************************/
void TIM1_BRK_TIM15_IRQHandler(void) {
    if ((LL_TIM_IsActiveFlag_UPDATE(TIM15) != RESET) && (LL_TIM_IsEnabledIT_UPDATE(TIM15) != RESET)) {
        LL_TIM_ClearFlag_UPDATE(TIM15);

//        printf("\r\n__UPDATE 溢出__\r\n");

        if (LL_TIM_IsActiveFlag_CC1(TIM15) != SET) {
            g_PWM_IN.CC1_OverflowCnt++;
        }

        if (LL_TIM_IsActiveFlag_CC2(TIM15) != SET) {
            g_PWM_IN.CC2_OverflowCnt++;
        }
    }

    if ((LL_TIM_IsActiveFlag_CC1(TIM15) != RESET) && (LL_TIM_IsEnabledIT_CC1(TIM15) != RESET)) {
        LL_TIM_ClearFlag_CC1(TIM15);

//        g_TIM_IT_CNT.TIM15_CC1++;
        g_PWM_IN.PulseWidth = (LL_TIM_IC_GetCaptureCH1(TIM15) + 1) + 65536 * g_PWM_IN.CC1_OverflowCnt;
    }

    if ((LL_TIM_IsActiveFlag_CC2(TIM15) != RESET) && (LL_TIM_IsEnabledIT_CC2(TIM15) != RESET)) {
        LL_TIM_ClearFlag_CC2(TIM15);

//        g_TIM_IT_CNT.TIM15_CC2++;

        g_PWM_IN.Period = (LL_TIM_IC_GetCaptureCH2(TIM15) + 1) + 65536 * g_PWM_IN.CC2_OverflowCnt;

        if (g_PWM_IN.PulseWidth <= g_PWM_IN.Period) {
            g_PWM_IN.DutyCycle = (g_PWM_IN.PulseWidth * 100) / g_PWM_IN.Period;
            g_PWM_IN.Frequency = F_SAMPLING / g_PWM_IN.Period;

//            printf("%lu 占空比：\t%lu %%\t频率：\t%lu Hz\r\n", g_Cnt++, g_PWM_IN.DutyCycle, g_PWM_IN.Frequency);
        } else {
            g_PWM_IN.DutyCycle = 0;
            g_PWM_IN.Frequency = 0;
        }

        g_PWM_IN.CC2_OverflowCnt = 0;
        g_PWM_IN.CC1_OverflowCnt = 0;
    }

    if ((LL_TIM_IsActiveFlag_CC1OVR(TIM15) != RESET) && (LL_TIM_IsEnabledIT_CC1(TIM15) != RESET)) {
        LL_TIM_ClearFlag_CC1OVR(TIM15);

        g_TIM_IT_CNT.TIM15_CC1++;

//        printf("\r\n__CC1 捕获溢出__\r\n");
    }

    if ((LL_TIM_IsActiveFlag_CC2OVR(TIM15) != RESET) && (LL_TIM_IsEnabledIT_CC2(TIM15) != RESET)) {
        LL_TIM_ClearFlag_CC2OVR(TIM15);

        g_TIM_IT_CNT.TIM15_CC2++;

//        printf("\r\n__CC2 捕获溢出__\r\n");
    }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

    /* System interrupt init*/

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM15_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    LL_RCC_GetSystemClocksFreq(&RCC_Clocks);

    printf("------------ CLK Freq ------------\r\n");
    printf("SYSCLK_Frequency:\t%lu MHz\r\n", RCC_Clocks.SYSCLK_Frequency / 1000000);
    printf("HCLK_Frequency:\t\t%lu MHz\r\n", RCC_Clocks.HCLK_Frequency / 1000000);
    printf("PCLK1_Frequency:\t%lu MHz\r\n", RCC_Clocks.PCLK1_Frequency / 1000000);
    printf("PCLK2_Frequency:\t%lu MHz\r\n", RCC_Clocks.PCLK2_Frequency / 1000000);
    printf("----------------------------------\r\n\r\n\r\n");

    GTek_PWM_InputMode_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        LL_mDelay(100);
        printf("%lu 占空比：\t%lu %%\t频率：\t%lu Hz\r\n", g_Cnt++, g_PWM_IN.DutyCycle, g_PWM_IN.Frequency);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        Error_Handler();
    }
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {

    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {

    }
    LL_Init1msTick(64000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(64000000);
    LL_RCC_SetTIMClockSource(LL_RCC_TIM15_CLKSOURCE_PCLK2);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(char *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
