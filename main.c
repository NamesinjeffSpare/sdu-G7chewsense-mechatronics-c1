#include "main.h"
#include "spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#define SAMPLE_RATE_HZ   20000
#define BUF_SAMPLES      256
#define CNVST_Port       GPIOA
#define CNVST_Pin        GPIO_PIN_1

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;

static volatile uint16_t sampleBuf[BUF_SAMPLES];
static volatile uint16_t writeIdx = 0;
static volatile uint8_t bufReady = 0;

static inline uint16_t MCP33131_ReadSample(void)
{
    uint16_t tx = 0x0000;
    uint16_t rx = 0x0000;

    HAL_GPIO_WritePin(CNVST_Port, CNVST_Pin, GPIO_PIN_SET);

    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    HAL_GPIO_WritePin(CNVST_Port, CNVST_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_TransmitReceive(&hspi1,
                                (uint8_t*)&tx,
                                (uint8_t*)&rx,
                                1, 10) != HAL_OK)
    {
        return 0;
    }

    rx = (rx >> 8) | (rx << 8);

    return rx;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uint16_t v = MCP33131_ReadSample();
        sampleBuf[writeIdx++] = v;

        if (writeIdx >= BUF_SAMPLES)
        {
            writeIdx = 0;
            bufReady = 1;
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USB_DEVICE_Init();
    MX_TIM2_Init();

    HAL_GPIO_WritePin(CNVST_Port, CNVST_Pin, GPIO_PIN_RESET);

    HAL_TIM_Base_Start_IT(&htim2);

    while (1)
    {
        if (bufReady)
        {
            bufReady = 0;

            char out[BUF_SAMPLES * 8];
            int idx = 0;

            for (int i = 0; i < BUF_SAMPLES; i++)
            {
                idx += sprintf(&out[idx], "%u\n", sampleBuf[i]);
            }

            CDC_Transmit_FS((uint8_t*)out, idx);
        }
    }
}

void SystemClock_Config(void)
{
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = CNVST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CNVST_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    while (1) { }
}  