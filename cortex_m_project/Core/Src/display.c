#include "display.h"

#include "main.h"
#include "spi.h"

#include <stddef.h>
#include "cmsis_os.h"

extern uint8_t current_mode;

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

/* ------------------------------------------------------------------
 * 장착 방향 보정
 *
 * LCD 패널이 기구에 180° 돌아간 상태로 붙어 있다. MADCTL 을 landscape
 * 기본값(0x28)으로 두면 버퍼의 (0,0) 이 실제로는 화면 우측 하단에 찍히므로
 * 우측 상단에 그린 FPS 가 좌측 하단에 거꾸로 나온다.
 *
 * MADCTL 에 MY|MX 를 더해(0xE8) 패널의 스캔 방향 자체를 뒤집으면 소프트웨어는
 * (0,0) 이 실제 좌측 상단인 좌표계를 그대로 쓸 수 있다. 오버레이든 나중에
 * 추가할 바운딩 박스든 개별 보정이 필요 없고 CPU 비용도 0 이다.
 *
 * ILI9341 MADCTL 비트 : MY 0x80 | MX 0x40 | MV 0x20 | ML 0x10 | BGR 0x08
 *   0x28 = MV|BGR                 landscape
 *   0xE8 = MY|MX|MV|BGR           landscape 180° 회전   <-- 현재 사용
 * ------------------------------------------------------------------ */
#define DISPLAY_MADCTL_VALUE      0xE8U

/*
 * 카메라 모듈도 180° 돌아간 상태로 붙어 있다.
 *
 * 지금까지는 카메라 뒤집힘과 패널 뒤집힘이 서로 상쇄되어 영상만은 똑바로
 * 보였다. 위에서 패널을 바로잡았으므로 카메라 쪽 180° 를 여기서 상쇄한다.
 * 원본을 복사하지 않고 읽는 좌표만 뒤집으므로 역시 추가 비용이 없다.
 *
 * 만약 이 상태에서 영상만 거꾸로 나온다면 카메라는 정상 장착이라는 뜻이므로
 * 이 값을 0 으로 바꾸면 된다. 오버레이 위치는 영향받지 않는다.
 */
#define DISPLAY_CAMERA_FLIP_180   1

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
     * 0xE8:
     * - Landscape orientation (320 x 240)
     * - 패널이 180° 돌아 장착되어 있어 MY|MX 로 스캔 방향을 뒤집는다
     * - BGR color order
     *
     * 자세한 내용은 파일 상단 DISPLAY_MADCTL_VALUE 주석 참고
     */
	data[0] = DISPLAY_MADCTL_VALUE;

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


/* ==================================================================
 * 오버레이 UI
 *
 * Display_UpdateImage() 는 프레임버퍼 없이 한 스캔라인(320px, 640byte)씩
 * 만들어 SPI DMA 로 곧바로 흘려보낸다. 따라서 오버레이도 레이어를 따로
 * 두지 않고, 스캔라인 버퍼가 완성된 직후 DMA 전송 직전에 그 줄에 걸리는
 * 오버레이 픽셀만 덮어쓰는 방식으로 합성한다.
 * ================================================================== */

/* ------------------------------------------------------------------
 * 시스템 상태 / FPS 연동부
 *
 * 상태와 FPS 의 주인은 app_main 이다. display 는 읽기만 한다.
 * app_main 에 무엇을 선언해야 하는지는 display.h 의
 * OVERLAY_USE_APP_GLOBALS 주석에 정리해 두었다.
 *
 * OVERLAY_USE_APP_GLOBALS == 1 이면 display.h 가 app_main.h 를 include 하므로
 * g_current_fps / g_track_state 선언이 그대로 넘어온다. 여기서 다시 extern 을
 * 적을 필요가 없다.
 *
 * 아직 0 이므로 지금은 아래 내부 대체 변수를 쓴다.
 * (화면에는 IDLE 상태 초록 조준점 / FPS 0.0 으로 고정 표시된다)
 * ------------------------------------------------------------------ */
#if (OVERLAY_USE_APP_GLOBALS == 0)

static volatile float   g_current_fps = 0.0f;
static volatile uint8_t g_track_state = (uint8_t)MACHINE_STATE_IDLE;

#endif


/* 조준점 형상 : 십자선 + 중앙 갭 (전체 48x48px, 선 두께 2px, 갭 12px) */
#define OVERLAY_CENTER_X       (DISPLAY_WIDTH / 2U)             /* 160 */
#define OVERLAY_CENTER_Y       (DISPLAY_HEIGHT / 2U)            /* 120 */
#define OVERLAY_CROSS_ARM      24U                              /* 중심~끝 */
#define OVERLAY_CROSS_GAP      6U                               /* 중심~선 시작 */
#define OVERLAY_CROSS_THICK    2U
#define OVERLAY_CROSS_LEN      (OVERLAY_CROSS_ARM - OVERLAY_CROSS_GAP)

/* 선 두께 2px 이므로 중심에서 1px 앞에서 시작한다 */
#define OVERLAY_CROSS_X0       (OVERLAY_CENTER_X - (OVERLAY_CROSS_THICK / 2U))
#define OVERLAY_CROSS_Y0       (OVERLAY_CENTER_Y - (OVERLAY_CROSS_THICK / 2U))

/* 5x7 비트맵 폰트를 2배 스케일로 그린다 */
#define OVERLAY_FONT_W         5U
#define OVERLAY_FONT_H         7U
#define OVERLAY_TEXT_SCALE     2U
#define OVERLAY_CHAR_W         (OVERLAY_FONT_W * OVERLAY_TEXT_SCALE)   /* 10 */
#define OVERLAY_CHAR_H         (OVERLAY_FONT_H * OVERLAY_TEXT_SCALE)   /* 14 */
#define OVERLAY_CHAR_GAP       2U
#define OVERLAY_CHAR_ADVANCE   (OVERLAY_CHAR_W + OVERLAY_CHAR_GAP)     /* 12 */

/* "FPS 999.9" + '\0' = 10자 */
#define OVERLAY_TEXT_MAX       12U
#define OVERLAY_TEXT_MARGIN_X  6
#define OVERLAY_TEXT_MARGIN_Y  6

/*
 * 스캔라인 조기 종료용 상하 경계.
 * 240줄 중 오버레이가 걸리는 줄은 60줄 남짓이므로 나머지는 즉시 빠져나간다.
 * 검은 외곽선과 텍스트 그림자 때문에 1px 여유를 둔다.
 */
#define OVERLAY_CROSS_TOP      ((int32_t)OVERLAY_CENTER_Y - (int32_t)OVERLAY_CROSS_ARM - 1)
#define OVERLAY_CROSS_BOTTOM   ((int32_t)OVERLAY_CENTER_Y + (int32_t)OVERLAY_CROSS_ARM + 1)
#define OVERLAY_TEXT_TOP       ((int32_t)OVERLAY_TEXT_MARGIN_Y - 1)
#define OVERLAY_TEXT_BOTTOM    ((int32_t)OVERLAY_TEXT_MARGIN_Y + (int32_t)OVERLAY_CHAR_H + 1)

/* 조준점 색상 */
#define OVERLAY_COLOR_IDLE      DISPLAY_COLOR_GREEN
#define OVERLAY_COLOR_TRACKING  DISPLAY_COLOR_YELLOW
#define OVERLAY_COLOR_LOCKON    DISPLAY_COLOR_RED

/* 밝은 영상 위에서도 형상이 보이도록 깔아주는 외곽선/그림자 색 */
#define OVERLAY_COLOR_OUTLINE   DISPLAY_COLOR_BLACK
#define OVERLAY_COLOR_TEXT      DISPLAY_COLOR_WHITE


typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} OverlayRect_t;

/*
 * 십자선을 5개의 사각형으로 분해해 둔다.
 * (위 팔, 아래 팔, 왼쪽 팔, 오른쪽 팔, 중앙 도트)
 */
static const OverlayRect_t overlay_cross_rects[] =
{
    /* 위쪽 세로 팔 */
    { OVERLAY_CROSS_X0,
      OVERLAY_CENTER_Y - OVERLAY_CROSS_ARM,
      OVERLAY_CROSS_THICK,
      OVERLAY_CROSS_LEN },

    /* 아래쪽 세로 팔 */
    { OVERLAY_CROSS_X0,
      OVERLAY_CENTER_Y + OVERLAY_CROSS_GAP,
      OVERLAY_CROSS_THICK,
      OVERLAY_CROSS_LEN },

    /* 왼쪽 가로 팔 */
    { OVERLAY_CENTER_X - OVERLAY_CROSS_ARM,
      OVERLAY_CROSS_Y0,
      OVERLAY_CROSS_LEN,
      OVERLAY_CROSS_THICK },

    /* 오른쪽 가로 팔 */
    { OVERLAY_CENTER_X + OVERLAY_CROSS_GAP,
      OVERLAY_CROSS_Y0,
      OVERLAY_CROSS_LEN,
      OVERLAY_CROSS_THICK },

    /* 중앙 도트 */
    { OVERLAY_CROSS_X0,
      OVERLAY_CROSS_Y0,
      OVERLAY_CROSS_THICK,
      OVERLAY_CROSS_THICK },
};

#define OVERLAY_CROSS_RECT_COUNT \
    (sizeof(overlay_cross_rects) / sizeof(overlay_cross_rects[0]))


/*
 * 5x7 비트맵 폰트 (컬럼 단위, bit0 = 맨 윗줄)
 *
 * FPS 표시에 필요한 '0'~'9', '.', 'F', 'P', 'S', ' ' 만 담는다.
 */
static const uint8_t overlay_font5x7[][OVERLAY_FONT_W] =
{
    { 0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU },  /*  0 : '0' */
    { 0x00U, 0x42U, 0x7FU, 0x40U, 0x00U },  /*  1 : '1' */
    { 0x42U, 0x61U, 0x51U, 0x49U, 0x46U },  /*  2 : '2' */
    { 0x21U, 0x41U, 0x45U, 0x4BU, 0x31U },  /*  3 : '3' */
    { 0x18U, 0x14U, 0x12U, 0x7FU, 0x10U },  /*  4 : '4' */
    { 0x27U, 0x45U, 0x45U, 0x45U, 0x39U },  /*  5 : '5' */
    { 0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U },  /*  6 : '6' */
    { 0x01U, 0x71U, 0x09U, 0x05U, 0x03U },  /*  7 : '7' */
    { 0x36U, 0x49U, 0x49U, 0x49U, 0x36U },  /*  8 : '8' */
    { 0x06U, 0x49U, 0x49U, 0x29U, 0x1EU },  /*  9 : '9' */
    { 0x00U, 0x60U, 0x60U, 0x00U, 0x00U },  /* 10 : '.' */
    { 0x7FU, 0x09U, 0x09U, 0x09U, 0x01U },  /* 11 : 'F' */
    { 0x7FU, 0x09U, 0x09U, 0x09U, 0x06U },  /* 12 : 'P' */
    { 0x46U, 0x49U, 0x49U, 0x49U, 0x31U },  /* 13 : 'S' */
};

#define OVERLAY_GLYPH_DOT   10
#define OVERLAY_GLYPH_F     11
#define OVERLAY_GLYPH_P     12
#define OVERLAY_GLYPH_S     13


/* 오버레이 표시 여부 */
static volatile bool overlay_active = false;

/*
 * 프레임 단위로 확정되는 값들.
 *
 * 스캔라인을 그리는 도중 g_track_state / g_current_fps 가 바뀌어도
 * 화면 위아래가 서로 다른 상태로 섞이지 않도록, 프레임 시작 시점에
 * 한 번만 스냅샷을 떠서 아래 변수에 담아두고 그 값으로만 그린다.
 */
static uint16_t overlay_cross_color = OVERLAY_COLOR_IDLE;
static char     overlay_text[OVERLAY_TEXT_MAX];
static int32_t  overlay_text_x      = 0;


/**
 * @brief 문자 하나에 대응하는 폰트 글리프를 찾는다.
 *
 * @return 글리프 포인터. 그릴 필요가 없는 문자(공백/미지원)면 NULL.
 */
static const uint8_t *Overlay_GetGlyph(char character)
{
    if ((character >= '0') && (character <= '9'))
    {
        return overlay_font5x7[character - '0'];
    }

    switch (character)
    {
    case '.':
        return overlay_font5x7[OVERLAY_GLYPH_DOT];

    case 'F':
        return overlay_font5x7[OVERLAY_GLYPH_F];

    case 'P':
        return overlay_font5x7[OVERLAY_GLYPH_P];

    case 'S':
        return overlay_font5x7[OVERLAY_GLYPH_S];

    default:
        /* 공백을 포함해 그릴 것이 없는 문자 */
        return NULL;
    }
}


/**
 * @brief FPS 값을 "FPS 30.0" 형태의 문자열로 만든다.
 *
 * snprintf("%.1f") 를 쓰면 float 포맷팅 루틴이 통째로 링크되므로
 * 소수 첫째 자리까지 직접 정수 연산으로 만든다.
 *
 * @return 만들어진 문자열 길이
 */
static uint32_t Overlay_FormatFps(float fps, char *out)
{
    uint32_t scaled;
    uint32_t whole;
    uint32_t frac;
    uint32_t index = 0U;

    if (!(fps > 0.0f))
    {
        /* 음수와 NaN 을 함께 걸러낸다 */
        fps = 0.0f;
    }

    /* 소수 첫째 자리까지 반올림 */
    scaled = (uint32_t)((fps * 10.0f) + 0.5f);

    if (scaled > 9999U)
    {
        scaled = 9999U;   /* 표시 상한 999.9 */
    }

    whole = scaled / 10U;
    frac  = scaled % 10U;

    out[index++] = 'F';
    out[index++] = 'P';
    out[index++] = 'S';
    out[index++] = ' ';

    if (whole >= 100U)
    {
        out[index++] = (char)('0' + (whole / 100U));
    }

    if (whole >= 10U)
    {
        out[index++] = (char)('0' + ((whole / 10U) % 10U));
    }

    out[index++] = (char)('0' + (whole % 10U));
    out[index++] = '.';
    out[index++] = (char)('0' + frac);
    out[index]   = '\0';

    return index;
}


/**
 * @brief 스캔라인 버퍼의 [x, x + width) 구간을 한 색으로 채운다.
 *
 * 화면 밖으로 나가는 부분은 잘라낸다.
 */
static void Overlay_FillSpan(uint8_t *line,
                             int32_t x,
                             int32_t width,
                             uint16_t color)
{
    int32_t x_end = x + width;

    if (x < 0)
    {
        x = 0;
    }

    if (x_end > (int32_t)DISPLAY_WIDTH)
    {
        x_end = (int32_t)DISPLAY_WIDTH;
    }

    for (; x < x_end; x++)
    {
        /* Display_UpdateImage 와 동일하게 Big-Endian 으로 적재한다 */
        line[x * 2]       = (uint8_t)(color >> 8);
        line[(x * 2) + 1] = (uint8_t)(color & 0xFFU);
    }
}


/**
 * @brief 사각형 중 현재 스캔라인에 걸리는 부분을 그린다.
 *
 * @param grow  사각형을 사방으로 넓힐 픽셀 수. 외곽선을 그릴 때 1을 준다.
 */
static void Overlay_DrawRectLine(uint8_t *line,
                                 int32_t y,
                                 const OverlayRect_t *rect,
                                 int32_t grow,
                                 uint16_t color)
{
    int32_t rect_y      = (int32_t)rect->y - grow;
    int32_t rect_height = (int32_t)rect->h + (2 * grow);

    if ((y < rect_y) || (y >= (rect_y + rect_height)))
    {
        return;
    }

    Overlay_FillSpan(line,
                     (int32_t)rect->x - grow,
                     (int32_t)rect->w + (2 * grow),
                     color);
}


/**
 * @brief 글리프 하나 중 현재 스캔라인에 걸리는 부분을 그린다.
 */
static void Overlay_DrawGlyphLine(uint8_t *line,
                                  int32_t y,
                                  int32_t glyph_x,
                                  int32_t glyph_y,
                                  const uint8_t *glyph,
                                  uint16_t color)
{
    int32_t font_row;
    uint32_t column;

    if (y < glyph_y)
    {
        return;
    }

    /* 2배 스케일이므로 화면 2줄이 폰트 1줄에 대응한다 */
    font_row = (y - glyph_y) / (int32_t)OVERLAY_TEXT_SCALE;

    if (font_row >= (int32_t)OVERLAY_FONT_H)
    {
        return;
    }

    for (column = 0U; column < OVERLAY_FONT_W; column++)
    {
        if (((glyph[column] >> font_row) & 0x01U) != 0U)
        {
            Overlay_FillSpan(line,
                             glyph_x + (int32_t)(column * OVERLAY_TEXT_SCALE),
                             (int32_t)OVERLAY_TEXT_SCALE,
                             color);
        }
    }
}


/**
 * @brief 문자열 중 현재 스캔라인에 걸리는 부분을 그린다.
 */
static void Overlay_DrawTextLine(uint8_t *line,
                                 int32_t y,
                                 int32_t text_x,
                                 int32_t text_y,
                                 const char *text,
                                 uint16_t color)
{
    uint32_t index;

    for (index = 0U; text[index] != '\0'; index++)
    {
        const uint8_t *glyph = Overlay_GetGlyph(text[index]);

        if (glyph != NULL)
        {
            Overlay_DrawGlyphLine(line,
                                  y,
                                  text_x + (int32_t)(index * OVERLAY_CHAR_ADVANCE),
                                  text_y,
                                  glyph,
                                  color);
        }
    }
}


/**
 * @brief 프레임 시작 시 상태/FPS 를 스냅샷하고 그릴 내용을 확정한다.
 */
static void Overlay_BeginFrame(void)
{
    uint8_t  state  = g_track_state;
    float    fps    = g_current_fps;
    uint32_t length;
    int32_t  width;

    switch (state)
    {
    case MACHINE_STATE_TRACKING:
        overlay_cross_color = OVERLAY_COLOR_TRACKING;
        break;

    case MACHINE_STATE_LOCKON:
        overlay_cross_color = OVERLAY_COLOR_LOCKON;
        break;

    case MACHINE_STATE_IDLE:
    default:
        overlay_cross_color = OVERLAY_COLOR_IDLE;
        break;
    }

    length = Overlay_FormatFps(fps, overlay_text);

    /* 마지막 글자 뒤의 자간은 폭에서 빼고 우측 정렬한다 */
    width = (int32_t)(length * OVERLAY_CHAR_ADVANCE) - (int32_t)OVERLAY_CHAR_GAP;

    overlay_text_x = (int32_t)DISPLAY_WIDTH - OVERLAY_TEXT_MARGIN_X - width;
}


/**
 * @brief 완성된 스캔라인 버퍼 위에 오버레이를 덮어쓴다.
 *
 * @param line 320픽셀(640바이트) 분량의 RGB565 Big-Endian 버퍼
 * @param y    화면 세로 좌표
 */
static void Overlay_RenderLine(uint8_t *line, uint16_t y)
{
    int32_t line_y = (int32_t)y;
    uint32_t index;

    /* 오버레이가 없는 줄은 즉시 빠져나간다 (240줄 중 대부분) */
    if (((line_y < OVERLAY_CROSS_TOP) || (line_y > OVERLAY_CROSS_BOTTOM)) &&
        ((line_y < OVERLAY_TEXT_TOP)  || (line_y > OVERLAY_TEXT_BOTTOM)))
    {
        return;
    }

    /*
     * 십자선은 검은 외곽선을 먼저 전부 깔고 나서 본체를 얹는다.
     * 사각형마다 외곽선-본체를 번갈아 그리면 옆 사각형의 외곽선이
     * 이미 그린 본체를 덮어버린다.
     */
    for (index = 0U; index < OVERLAY_CROSS_RECT_COUNT; index++)
    {
        Overlay_DrawRectLine(line,
                             line_y,
                             &overlay_cross_rects[index],
                             1,
                             OVERLAY_COLOR_OUTLINE);
    }

    for (index = 0U; index < OVERLAY_CROSS_RECT_COUNT; index++)
    {
        Overlay_DrawRectLine(line,
                             line_y,
                             &overlay_cross_rects[index],
                             0,
                             overlay_cross_color);
    }

    /* FPS 텍스트도 그림자를 먼저 깔고 본문을 얹는다 */
    Overlay_DrawTextLine(line,
                         line_y,
                         overlay_text_x + 1,
                         OVERLAY_TEXT_MARGIN_Y + 1,
                         overlay_text,
                         OVERLAY_COLOR_OUTLINE);

    Overlay_DrawTextLine(line,
                         line_y,
                         overlay_text_x,
                         OVERLAY_TEXT_MARGIN_Y,
                         overlay_text,
                         OVERLAY_COLOR_TEXT);
}


void ActivateOverlayWidget(bool enable)
{
    overlay_active = enable;
}


bool IsOverlayWidgetActive(void)
{
    return overlay_active;
}


// SPI DMA 전송 완료 대기를 위한 플래그 또는 상태 확인용 변수
volatile uint8_t display_dma_completed = 0;

// YUV 모드
//HAL_StatusTypeDef Display_UpdateImage(const uint16_t *image, uint16_t width,
//		uint16_t height) {
//	HAL_StatusTypeDef status;
//	uint16_t dst_x;
//	uint16_t dst_y;
//	uint32_t src_x;
//	uint32_t src_y;
//	uint32_t src_index;
//
//	if (image == NULL || width == 0U || height == 0U) {
//		return HAL_ERROR;
//	}
//
//	if ((width > DISPLAY_WIDTH) || (height > DISPLAY_HEIGHT)) {
//		return HAL_ERROR;
//	}
//
//	// 1. STM32H7 캐시 동기화 (카메라로 받은 원본 이미지 메모리 무효화)
//	SCB_InvalidateDCache_by_Addr((uint32_t*) image, width * height * 2);
//
//	status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U,
//	DISPLAY_HEIGHT - 1U);
//	if (status != HAL_OK) {
//		return status;
//	}
//
//	DISPLAY_CS_LOW();
//	DISPLAY_DC_DATA();
//
//	uint8_t *p_bytes = (uint8_t*) image;
//
//	for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
//		src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;
//
//		for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
//			src_x = ((uint32_t) dst_x * width) / DISPLAY_WIDTH;
//			src_index = (src_y * width) + src_x;
//
//			// current_mode가 1(RGB)일 때
//			if (current_mode == 1) {
//				uint16_t pixel = image[src_index];
//				display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
//				display_tx_buffer[(dst_x * 2U) + 1U] =
//						(uint8_t) (pixel & 0xFFU);
//			}
//			// current_mode가 0(Grayscale)일 때
//			else {
//				uint8_t *p_bytes = (uint8_t*) image;
//				uint8_t y_val = p_bytes[src_index * 2U + 1U];
//
//				uint16_t r = (y_val >> 3) & 0x1FU;
//				uint16_t g = (y_val >> 2) & 0x3FU;
//				uint16_t b = (y_val >> 3) & 0x1FU;
//				uint16_t pixel = (r << 11) | (g << 5) | b;
//
//				display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
//				display_tx_buffer[(dst_x * 2U) + 1U] =
//						(uint8_t) (pixel & 0xFFU);
//			}
//		}
//
//		// 2. 전송할 텍스처 버퍼 캐시 클린 (메모리에 확실히 써지도록 함)
//		SCB_CleanDCache_by_Addr((uint32_t*) display_tx_buffer,
//		DISPLAY_WIDTH * 2U);
//
//		// 3. SPI DMA 전송 시작
//		display_dma_completed = 0;
//		if (HAL_SPI_Transmit_DMA(&hspi2, display_tx_buffer, DISPLAY_WIDTH * 2U)
//				!= HAL_OK) {
//			DISPLAY_CS_HIGH();
//			return HAL_ERROR;
//		}
//
//		// 4. DMA 전송이 완료될 때까지 대기
//		while (display_dma_completed == 0) {
//			// 대기 중 칩셋 부하를 줄이려면 __WFI(); 삽입 가능
//		}
//	}
//
//	DISPLAY_CS_HIGH();
//	return HAL_OK;
//}

// rgb565 mode
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

	// STM32H7 캐시 동기화 (카메라로 받은 원본 이미지 메모리 무효화)
	SCB_InvalidateDCache_by_Addr((uint32_t*) image, width * height * 2);

	status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U,
	DISPLAY_HEIGHT - 1U);
	if (status != HAL_OK) {
		return status;
	}

	DISPLAY_CS_LOW();
	DISPLAY_DC_DATA();

	uint8_t *p_bytes = (uint8_t*) image;

	// 이번 프레임에 그릴 오버레이 내용(상태 색상 / FPS 문자열) 확정
	if (overlay_active) {
		Overlay_BeginFrame();
	}

	for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
		src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;

#if (DISPLAY_CAMERA_FLIP_180 == 1)
		// 카메라 180° 장착 상쇄 (세로)
		src_y = (uint32_t) (height - 1U) - src_y;
#endif

		for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
			src_x = ((uint32_t) dst_x * width) / DISPLAY_WIDTH;

#if (DISPLAY_CAMERA_FLIP_180 == 1)
			// 카메라 180° 장착 상쇄 (가로)
			src_x = (uint32_t) (width - 1U) - src_x;
#endif

			src_index = (src_y * width) + src_x;

			// 카메라가 보낸 RGB565 픽셀 데이터 원본 그대로 사용
			uint16_t pixel = image[src_index];

			// LCD로 전송 (Big-Endian 순서)
			display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);   // 상위 바이트
			display_tx_buffer[(dst_x * 2U) + 1U] = (uint8_t) (pixel & 0xFFU); // 하위 바이트
		}

		// 완성된 영상 스캔라인 위에 오버레이 UI를 덮어씀
		if (overlay_active) {
			Overlay_RenderLine(display_tx_buffer, dst_y);
		}

		// 전송할 텍스처 버퍼 캐시 클린 (메모리에 확실히 써지도록 함)
		SCB_CleanDCache_by_Addr((uint32_t*) display_tx_buffer,
		DISPLAY_WIDTH * 2U);

		// SPI DMA 전송 시작
		display_dma_completed = 0;
		if (HAL_SPI_Transmit_DMA(&hspi2, display_tx_buffer, DISPLAY_WIDTH * 2U)
				!= HAL_OK) {
			DISPLAY_CS_HIGH();
			return HAL_ERROR;
		}

		// DMA 전송이 완료될 때까지 대기
		while (display_dma_completed == 0) {
			// 대기 중 칩셋 부하를 줄이려면 __WFI(); 삽입 가능
		}
	}

	DISPLAY_CS_HIGH();
	return HAL_OK;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI2) {
		display_dma_completed = 1;
	}
}
