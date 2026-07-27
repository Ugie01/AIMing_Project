#include "display.h"

#include "main.h"
#include "spi.h"

#include <stddef.h>

/* ILI9341 commands */
#define ILI9341_CMD_SWRESET   0x01U
#define ILI9341_CMD_SLPOUT    0x11U
#define ILI9341_CMD_DISPON    0x29U
#define ILI9341_CMD_CASET     0x2AU
#define ILI9341_CMD_PASET     0x2BU
#define ILI9341_CMD_RAMWR     0x2CU
#define ILI9341_CMD_MADCTL    0x36U
#define ILI9341_CMD_PIXFMT    0x3AU

#define DISPLAY_SPI_TIMEOUT_MS    1000U

/*
 * 512바이트 = RGB565 픽셀 256개
 *
 * 지역변수로 만들면 스택을 많이 사용하므로
 * display.c 내부 정적 버퍼로 둔다.
 */
#define DISPLAY_TX_BUFFER_SIZE    1024U

static uint8_t display_tx_buffer[DISPLAY_TX_BUFFER_SIZE];

/*
 * CubeMX User Label이 아래 이름과 정확히 같아야 한다.
 *
 * PB10 = TFT_RST
 * PB11 = TFT_DC
 * PB12 = TFT_CS
 */
#define DISPLAY_CS_LOW()   \
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)

#define DISPLAY_CS_HIGH()  \
    HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)

#define DISPLAY_DC_COMMAND() \
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET)

#define DISPLAY_DC_DATA() \
    HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET)

#define DISPLAY_RST_LOW() \
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET)

#define DISPLAY_RST_HIGH() \
    HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET)


extern SPI_HandleTypeDef hspi2;

/**
 * @brief SPI2로 데이터를 송신한다.
 */
static HAL_StatusTypeDef Display_SPITransmit(const uint8_t *data,
                                             uint16_t size)
{
    if ((data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * HAL 함수가 uint8_t *를 요구하므로 const를 캐스팅한다.
     * 함수 내부에서 송신 데이터 자체를 수정하지는 않는다.
     */
    return HAL_SPI_Transmit(&hspi2,
                            (uint8_t *)data,
                            size,
                            DISPLAY_SPI_TIMEOUT_MS);
}


/**
 * @brief 명령 1바이트를 전송한다.
 */
static HAL_StatusTypeDef Display_WriteCommand(uint8_t command)
{
    HAL_StatusTypeDef status;

    DISPLAY_CS_LOW();
    DISPLAY_DC_COMMAND();

    status = Display_SPITransmit(&command, 1U);

    DISPLAY_CS_HIGH();

    return status;
}


/**
 * @brief 명령과 뒤따르는 데이터를 한 트랜잭션으로 전송한다.
 */
static HAL_StatusTypeDef Display_WriteCommandData(uint8_t command,
                                                  const uint8_t *data,
                                                  uint16_t size)
{
    HAL_StatusTypeDef status;

    DISPLAY_CS_LOW();

    DISPLAY_DC_COMMAND();

    status = Display_SPITransmit(&command, 1U);

    if ((status == HAL_OK) && (data != NULL) && (size > 0U))
    {
        DISPLAY_DC_DATA();

        status = Display_SPITransmit(data, size);
    }

    DISPLAY_CS_HIGH();

    return status;
}


/**
 * @brief TFT 하드웨어 리셋
 */
static void Display_HardwareReset(void)
{
    DISPLAY_CS_HIGH();
    DISPLAY_RST_HIGH();

    HAL_Delay(10U);

    DISPLAY_RST_LOW();

    HAL_Delay(20U);

    DISPLAY_RST_HIGH();

    /*
     * 리셋 해제 후 내부 회로가 안정화될 시간을 준다.
     */
    HAL_Delay(150U);
}


/**
 * @brief LCD 내부 GRAM에 접근할 영역을 지정한다.
 */
static HAL_StatusTypeDef Display_SetAddressWindow(uint16_t x_start,
                                                  uint16_t y_start,
                                                  uint16_t x_end,
                                                  uint16_t y_end)
{
    HAL_StatusTypeDef status;
    uint8_t address_data[4];

    /* Column address */
    address_data[0] = (uint8_t)(x_start >> 8);
    address_data[1] = (uint8_t)(x_start & 0xFFU);
    address_data[2] = (uint8_t)(x_end >> 8);
    address_data[3] = (uint8_t)(x_end & 0xFFU);

    status = Display_WriteCommandData(ILI9341_CMD_CASET,
                                      address_data,
                                      sizeof(address_data));

    if (status != HAL_OK)
    {
        return status;
    }

    /* Page/row address */
    address_data[0] = (uint8_t)(y_start >> 8);
    address_data[1] = (uint8_t)(y_start & 0xFFU);
    address_data[2] = (uint8_t)(y_end >> 8);
    address_data[3] = (uint8_t)(y_end & 0xFFU);

    status = Display_WriteCommandData(ILI9341_CMD_PASET,
                                      address_data,
                                      sizeof(address_data));

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * 이후 전송하는 데이터는 화면 픽셀 데이터로 처리된다.
     */
    return Display_WriteCommand(ILI9341_CMD_RAMWR);
}


HAL_StatusTypeDef Display_Init(void)
{
    HAL_StatusTypeDef status;

    uint8_t data[15];
	HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_SET);
    Display_HardwareReset();

    /*
     * Software Reset
     */
    status = Display_WriteCommand(ILI9341_CMD_SWRESET);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(150U);

    /*
     * Power control B
     */
    data[0] = 0x00U;
    data[1] = 0xC1U;
    data[2] = 0x30U;

    status = Display_WriteCommandData(0xCFU, data, 3U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Power on sequence control
     */
    data[0] = 0x64U;
    data[1] = 0x03U;
    data[2] = 0x12U;
    data[3] = 0x81U;

    status = Display_WriteCommandData(0xEDU, data, 4U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Driver timing control A
     */
    data[0] = 0x85U;
    data[1] = 0x00U;
    data[2] = 0x78U;

    status = Display_WriteCommandData(0xE8U, data, 3U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Power control A
     */
    data[0] = 0x39U;
    data[1] = 0x2CU;
    data[2] = 0x00U;
    data[3] = 0x34U;
    data[4] = 0x02U;

    status = Display_WriteCommandData(0xCBU, data, 5U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Pump ratio control
     */
    data[0] = 0x20U;

    status = Display_WriteCommandData(0xF7U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Driver timing control B
     */
    data[0] = 0x00U;
    data[1] = 0x00U;

    status = Display_WriteCommandData(0xEAU, data, 2U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Power Control 1
     */
    data[0] = 0x23U;

    status = Display_WriteCommandData(0xC0U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Power Control 2
     */
    data[0] = 0x10U;

    status = Display_WriteCommandData(0xC1U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * VCOM Control 1
     */
    data[0] = 0x3EU;
    data[1] = 0x28U;

    status = Display_WriteCommandData(0xC5U, data, 2U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * VCOM Control 2
     */
    data[0] = 0x86U;

    status = Display_WriteCommandData(0xC7U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Memory Access Control
     *
     * 0x48:
     * - Portrait orientation
     * - BGR color order
     * - 240 x 320
     */
	data[0] = 0x28U;

    status = Display_WriteCommandData(ILI9341_CMD_MADCTL,
                                      data,
                                      1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Pixel Format
     *
     * 0x55 = 16bit RGB565
     */
    data[0] = 0x55U;

    status = Display_WriteCommandData(ILI9341_CMD_PIXFMT,
                                      data,
                                      1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Frame Rate Control
     */
    data[0] = 0x00U;
    data[1] = 0x18U;

    status = Display_WriteCommandData(0xB1U, data, 2U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Display Function Control
     */
    data[0] = 0x08U;
    data[1] = 0x82U;
    data[2] = 0x27U;

    status = Display_WriteCommandData(0xB6U, data, 3U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Disable 3-gamma function
     */
    data[0] = 0x00U;

    status = Display_WriteCommandData(0xF2U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Gamma curve
     */
    data[0] = 0x01U;

    status = Display_WriteCommandData(0x26U, data, 1U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Positive Gamma Correction
     */
    data[0]  = 0x0FU;
    data[1]  = 0x31U;
    data[2]  = 0x2BU;
    data[3]  = 0x0CU;
    data[4]  = 0x0EU;
    data[5]  = 0x08U;
    data[6]  = 0x4EU;
    data[7]  = 0xF1U;
    data[8]  = 0x37U;
    data[9]  = 0x07U;
    data[10] = 0x10U;
    data[11] = 0x03U;
    data[12] = 0x0EU;
    data[13] = 0x09U;
    data[14] = 0x00U;

    status = Display_WriteCommandData(0xE0U, data, 15U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Negative Gamma Correction
     */
    data[0]  = 0x00U;
    data[1]  = 0x0EU;
    data[2]  = 0x14U;
    data[3]  = 0x03U;
    data[4]  = 0x11U;
    data[5]  = 0x07U;
    data[6]  = 0x31U;
    data[7]  = 0xC1U;
    data[8]  = 0x48U;
    data[9]  = 0x08U;
    data[10] = 0x0FU;
    data[11] = 0x0CU;
    data[12] = 0x31U;
    data[13] = 0x36U;
    data[14] = 0x0FU;

    status = Display_WriteCommandData(0xE1U, data, 15U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Sleep Out
     */
    status = Display_WriteCommand(ILI9341_CMD_SLPOUT);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(120U);

    /*
     * Display On
     */
    status = Display_WriteCommand(ILI9341_CMD_DISPON);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(20U);

    return HAL_OK;
}


// SPI DMA 전송 완료 대기를 위한 플래그 또는 상태 확인용 변수
volatile uint8_t display_dma_completed = 0;

HAL_StatusTypeDef Display_UpdateImage(const uint16_t *image, uint16_t width,
		uint16_t height) {
	HAL_StatusTypeDef status;
	uint16_t dst_x;
	uint16_t dst_y;
	uint32_t src_x;
	uint32_t src_y;
	uint32_t src_index;

	if (image == NULL || width == 0U || height == 0U) {
		return HAL_ERROR;
	}

	if ((width > DISPLAY_WIDTH) || (height > DISPLAY_HEIGHT)) {
		return HAL_ERROR;
	}

	// 1. STM32H7 캐시 동기화 (카메라로 받은 원본 이미지 메모리 무효화)
	SCB_InvalidateDCache_by_Addr((uint32_t*) image, width * height * 2);

	status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U,
			DISPLAY_HEIGHT - 1U);
	if (status != HAL_OK) {
		return status;
	}

	DISPLAY_CS_LOW();
	DISPLAY_DC_DATA();

	uint8_t *p_bytes = (uint8_t*) image;

	for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
		src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;

		for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
			src_x = ((uint32_t) dst_x * width) / DISPLAY_WIDTH;
			src_index = (src_y * width) + src_x;

			// 흑백/컬러 모드에 따른 픽셀 추출 방식 확인
			// (현재 그레이스케일 추출 로직이 들어있다면 이 부분을 유지)
			uint8_t y_val = p_bytes[src_index * 2U + 1U];

            uint16_t r = (y_val >> 3) & 0x1FU;
			uint16_t g = (y_val >> 2) & 0x3FU;
			uint16_t b = (y_val >> 3) & 0x1FU;

			uint16_t pixel = (r << 11) | (g << 5) | b;

			display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
			display_tx_buffer[(dst_x * 2U) + 1U] = (uint8_t) (pixel & 0xFFU);
		}

		// 2. 전송할 텍스처 버퍼 캐시 클린 (메모리에 확실히 써지도록 함)
		SCB_CleanDCache_by_Addr((uint32_t*) display_tx_buffer,
				DISPLAY_WIDTH * 2U);

		// 3. SPI DMA 전송 시작
		display_dma_completed = 0;
		if (HAL_SPI_Transmit_DMA(&hspi2, display_tx_buffer, DISPLAY_WIDTH * 2U)
				!= HAL_OK) {
			DISPLAY_CS_HIGH();
			return HAL_ERROR;
		}

		// 4. DMA 전송이 완료될 때까지 대기
		while (display_dma_completed == 0) {
			// 대기 중 칩셋 부하를 줄이려면 __WFI(); 삽입 가능
		}
	}

	DISPLAY_CS_HIGH();
	return HAL_OK;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI2)
	{
		display_dma_completed = 1;
	}
}

//HAL_StatusTypeDef Display_UpdateImage(
//    const uint16_t *image,
//    uint16_t width,
//    uint16_t height
//)
//{
//    HAL_StatusTypeDef status;
//    uint16_t dst_x;
//    uint16_t dst_y;
//    uint32_t src_x;
//    uint32_t src_y;
//    uint32_t src_index;
//    uint16_t pixel;
//
//	if (image == NULL || width == 0U || height == 0U)
//    {
//        return HAL_ERROR;
//    }
//
//	if ((width > DISPLAY_WIDTH) || (height > DISPLAY_HEIGHT))
//    {
//        return HAL_ERROR;
//    }
//
//	status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U,
//			DISPLAY_HEIGHT - 1U);
//    if (status != HAL_OK)
//    {
//        return status;
//    }
//
//    DISPLAY_CS_LOW();
//    DISPLAY_DC_DATA();
//
//	for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
//		src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;
//
//		for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
//			src_x = ((uint32_t) dst_x * width) / DISPLAY_WIDTH;
//			src_index = (src_y * width) + src_x;
//
//			// 카메라가 이미 RGB565로 주므로 별도의 YUV 변환 없이 픽셀을 그대로 가져옴
//			pixel = image[src_index];
//
//			// 하드웨어 엔디안 및 ILI9341 바이트 오더 매칭 (필요시 바이트 스왑)
//			// 상위/하위 바이트 배치 조정
//			display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
//			display_tx_buffer[(dst_x * 2U) + 1U] = (uint8_t) (pixel & 0xFFU);
//		}
//
//		status = Display_SPITransmit(display_tx_buffer, DISPLAY_WIDTH * 2U);
//		if (status != HAL_OK) {
//			DISPLAY_CS_HIGH();
//			return status;
//		}
//	}
//
//    DISPLAY_CS_HIGH();
//    return HAL_OK;
//}
