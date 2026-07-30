#include "display.h"
#include "main.h"
#include "spi.h"
#include <stddef.h>
#include "cmsis_os.h"

// 상태 정보를 읽어오기 위한 태스크 헤더
#include "task_vision.h"
#include "task_motor.h"

// ==============================================================================
// 매크로 및 상수 정의
// ==============================================================================

// ILI9341 LCD 제어 명령어
#define ILI9341_CMD_SWRESET   0x01U
#define ILI9341_CMD_SLPOUT    0x11U
#define ILI9341_CMD_DISPON    0x29U
#define ILI9341_CMD_CASET     0x2AU
#define ILI9341_CMD_PASET     0x2BU
#define ILI9341_CMD_RAMWR     0x2CU
#define ILI9341_CMD_MADCTL    0x36U
#define ILI9341_CMD_PIXFMT    0x3AU

#define DISPLAY_SPI_TIMEOUT_MS    1000U

// 화면 회전 설정 (현재 180도 뒤집힌 상태 보정용)
#define DISPLAY_MADCTL_VALUE      0xE8U
// 카메라 180도 회전 보정 (0: 사용 안함, 1: 상하좌우 반전)
#define DISPLAY_CAMERA_FLIP_180   0

// SPI DMA 전송을 위한 스캔라인 버퍼 사이즈 (512바이트 = RGB565 256픽셀)
#define DISPLAY_TX_BUFFER_SIZE    1024U

// 핀 제어 매크로 (CubeMX User Label과 일치해야 함)
#define DISPLAY_CS_LOW()   HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET)
#define DISPLAY_CS_HIGH()  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET)
#define DISPLAY_DC_COMMAND() HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET)
#define DISPLAY_DC_DATA()  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET)
#define DISPLAY_RST_LOW()  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET)
#define DISPLAY_RST_HIGH() HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET)

// 오버레이 UI 조준점 형상 설정 (전체 48x48px, 선 두께 2px, 갭 12px)
#define OVERLAY_CENTER_X       (DISPLAY_WIDTH / 2U)
#define OVERLAY_CENTER_Y       (DISPLAY_HEIGHT / 2U)
#define OVERLAY_CROSS_ARM      24U
#define OVERLAY_CROSS_GAP      6U
#define OVERLAY_CROSS_THICK    2U
#define OVERLAY_CROSS_LEN      (OVERLAY_CROSS_ARM - OVERLAY_CROSS_GAP)
#define OVERLAY_CROSS_X0       (OVERLAY_CENTER_X - (OVERLAY_CROSS_THICK / 2U))
#define OVERLAY_CROSS_Y0       (OVERLAY_CENTER_Y - (OVERLAY_CROSS_THICK / 2U))

// 오버레이 폰트 설정 (5x7 비트맵, 2배 스케일)
#define OVERLAY_FONT_W         5U
#define OVERLAY_FONT_H         7U
#define OVERLAY_TEXT_SCALE     2U
#define OVERLAY_CHAR_W         (OVERLAY_FONT_W * OVERLAY_TEXT_SCALE)
#define OVERLAY_CHAR_H         (OVERLAY_FONT_H * OVERLAY_TEXT_SCALE)
#define OVERLAY_CHAR_GAP       2U
#define OVERLAY_CHAR_ADVANCE   (OVERLAY_CHAR_W + OVERLAY_CHAR_GAP)

// 문자열 버퍼 설정 ("FPS 999.9" = 10자 + 여유)
#define OVERLAY_TEXT_MAX       12U
#define OVERLAY_TEXT_MARGIN_X  6
#define OVERLAY_TEXT_MARGIN_Y  6

// 스캔라인 조기 종료를 위한 렌더링 영역 상하 경계 설정
#define OVERLAY_CROSS_TOP      ((int32_t)OVERLAY_CENTER_Y - (int32_t)OVERLAY_CROSS_ARM - 1)
#define OVERLAY_CROSS_BOTTOM   ((int32_t)OVERLAY_CENTER_Y + (int32_t)OVERLAY_CROSS_ARM + 1)
#define OVERLAY_TEXT_TOP       ((int32_t)OVERLAY_TEXT_MARGIN_Y - 1)
#define OVERLAY_TEXT_BOTTOM    ((int32_t)OVERLAY_TEXT_MARGIN_Y + (int32_t)OVERLAY_CHAR_H + 1)

// 상태별 오버레이 색상
#define OVERLAY_COLOR_IDLE      DISPLAY_COLOR_GREEN
#define OVERLAY_COLOR_TRACKING  DISPLAY_COLOR_YELLOW
#define OVERLAY_COLOR_LOCKON    DISPLAY_COLOR_RED
#define OVERLAY_COLOR_MANUAL    DISPLAY_COLOR_CYAN
#define OVERLAY_COLOR_OUTLINE   DISPLAY_COLOR_BLACK
#define OVERLAY_COLOR_TEXT      DISPLAY_COLOR_WHITE

// ==============================================================================
// 모듈 전역/정적 변수 모음
// ==============================================================================

extern SPI_HandleTypeDef hspi2;

// SPI 통신용 송신 버퍼 (SRAM)
ALIGN_32BYTES(static uint8_t display_tx_buffer[2][DISPLAY_TX_BUFFER_SIZE]);

// SPI DMA 전송 완료 대기 플래그
volatile uint8_t display_dma_completed = 0;

// 오버레이 UI 표시 여부 및 텍스트 상태 변수
static volatile bool overlay_active = true;
static uint16_t overlay_cross_color = OVERLAY_COLOR_IDLE;
static char overlay_text[OVERLAY_TEXT_MAX];
static int32_t overlay_text_x = 0;

// ==============================================================================
// 🚀 스케일링 연산 최소화를 위한 룩업 테이블 (LUT)
// ==============================================================================
static uint32_t lut_x[DISPLAY_WIDTH];
static uint32_t lut_y[DISPLAY_HEIGHT];
static uint16_t last_width = 0;
static uint16_t last_height = 0;

// ==============================================================================
// 내부 구조체 및 폰트 데이터 정의
// ==============================================================================

// 사각형 좌표 구조체
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
} OverlayRect_t;

// 십자선 구성을 위한 5개 사각형 조각 정의
static const OverlayRect_t overlay_cross_rects[] = {
        { OVERLAY_CROSS_X0, OVERLAY_CENTER_Y - OVERLAY_CROSS_ARM, OVERLAY_CROSS_THICK, OVERLAY_CROSS_LEN }, // 위
        { OVERLAY_CROSS_X0, OVERLAY_CENTER_Y + OVERLAY_CROSS_GAP, OVERLAY_CROSS_THICK, OVERLAY_CROSS_LEN }, // 아래
        { OVERLAY_CENTER_X - OVERLAY_CROSS_ARM, OVERLAY_CROSS_Y0, OVERLAY_CROSS_LEN, OVERLAY_CROSS_THICK }, // 왼쪽
        { OVERLAY_CENTER_X + OVERLAY_CROSS_GAP, OVERLAY_CROSS_Y0, OVERLAY_CROSS_LEN, OVERLAY_CROSS_THICK }, // 오른쪽
        { OVERLAY_CROSS_X0, OVERLAY_CROSS_Y0, OVERLAY_CROSS_THICK, OVERLAY_CROSS_THICK }, // 중앙 도트
        };
#define OVERLAY_CROSS_RECT_COUNT (sizeof(overlay_cross_rects) / sizeof(overlay_cross_rects[0]))

// 5x7 비트맵 폰트 (숫자, '.', 'F', 'P', 'S')
static const uint8_t overlay_font5x7[][OVERLAY_FONT_W] = { { 0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU },  // '0'
        { 0x00U, 0x42U, 0x7FU, 0x40U, 0x00U },  // '1'
        { 0x42U, 0x61U, 0x51U, 0x49U, 0x46U },  // '2'
        { 0x21U, 0x41U, 0x45U, 0x4BU, 0x31U },  // '3'
        { 0x18U, 0x14U, 0x12U, 0x7FU, 0x10U },  // '4'
        { 0x27U, 0x45U, 0x45U, 0x45U, 0x39U },  // '5'
        { 0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U },  // '6'
        { 0x01U, 0x71U, 0x09U, 0x05U, 0x03U },  // '7'
        { 0x36U, 0x49U, 0x49U, 0x49U, 0x36U },  // '8'
        { 0x06U, 0x49U, 0x49U, 0x29U, 0x1EU },  // '9'
        { 0x00U, 0x60U, 0x60U, 0x00U, 0x00U },  // '.'
        { 0x7FU, 0x09U, 0x09U, 0x09U, 0x01U },  // 'F'
        { 0x7FU, 0x09U, 0x09U, 0x09U, 0x06U },  // 'P'
        { 0x46U, 0x49U, 0x49U, 0x49U, 0x31U },  // 'S'
        };
#define OVERLAY_GLYPH_DOT   10
#define OVERLAY_GLYPH_F     11
#define OVERLAY_GLYPH_P     12
#define OVERLAY_GLYPH_S     13

// ==============================================================================
// 하드웨어(SPI/LCD) 제어 내부 함수
// ==============================================================================

// SPI2 데이터 송신
static HAL_StatusTypeDef Display_SPITransmit(const uint8_t *data, uint16_t size) {
    if ((data == NULL) || (size == 0U))
        return HAL_ERROR;
    return HAL_SPI_Transmit(&hspi2, (uint8_t*) data, size, DISPLAY_SPI_TIMEOUT_MS);
}

// LCD에 명령어 1바이트 전송
static HAL_StatusTypeDef Display_WriteCommand(uint8_t command) {
    HAL_StatusTypeDef status;
    DISPLAY_CS_LOW();
    DISPLAY_DC_COMMAND();
    status = Display_SPITransmit(&command, 1U);
    DISPLAY_CS_HIGH();
    return status;
}

// LCD에 명령어와 데이터 연속 전송
static HAL_StatusTypeDef Display_WriteCommandData(uint8_t command, const uint8_t *data, uint16_t size) {
    HAL_StatusTypeDef status;
    DISPLAY_CS_LOW();
    DISPLAY_DC_COMMAND();
    status = Display_SPITransmit(&command, 1U);
    if ((status == HAL_OK) && (data != NULL) && (size > 0U)) {
        DISPLAY_DC_DATA();
        status = Display_SPITransmit(data, size);
    }
    DISPLAY_CS_HIGH();
    return status;
}

// LCD 하드웨어 리셋 핀 제어
static void Display_HardwareReset(void) {
    DISPLAY_CS_HIGH();
    DISPLAY_RST_HIGH();
    HAL_Delay(10U);
    DISPLAY_RST_LOW();
    HAL_Delay(20U);
    DISPLAY_RST_HIGH();
    HAL_Delay(150U); // 리셋 후 안정화 대기
}

// LCD 내부에 그릴 영역(Window) 지정
static HAL_StatusTypeDef Display_SetAddressWindow(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end) {
    HAL_StatusTypeDef status;
    uint8_t address_data[4];

    // X 좌표
    address_data[0] = (uint8_t)(x_start >> 8);
    address_data[1] = (uint8_t)(x_start & 0xFFU);
    address_data[2] = (uint8_t)(x_end >> 8);
    address_data[3] = (uint8_t)(x_end & 0xFFU);
    status = Display_WriteCommandData(ILI9341_CMD_CASET, address_data, sizeof(address_data));
    if (status != HAL_OK)
        return status;

    // Y 좌표
    address_data[0] = (uint8_t)(y_start >> 8);
    address_data[1] = (uint8_t)(y_start & 0xFFU);
    address_data[2] = (uint8_t)(y_end >> 8);
    address_data[3] = (uint8_t)(y_end & 0xFFU);
    status = Display_WriteCommandData(ILI9341_CMD_PASET, address_data, sizeof(address_data));
    if (status != HAL_OK)
        return status;

    // RAM 쓰기 준비
    return Display_WriteCommand(ILI9341_CMD_RAMWR);
}

// ==============================================================================
// 오버레이 UI 렌더링 함수
// ==============================================================================

// 문자 하나를 받아 해당 폰트 글리프 반환
static const uint8_t* Overlay_GetGlyph(char character) {
    if ((character >= '0') && (character <= '9'))
        return overlay_font5x7[character - '0'];
    switch (character) {
    case '.':
        return overlay_font5x7[OVERLAY_GLYPH_DOT];
    case 'F':
        return overlay_font5x7[OVERLAY_GLYPH_F];
    case 'P':
        return overlay_font5x7[OVERLAY_GLYPH_P];
    case 'S':
        return overlay_font5x7[OVERLAY_GLYPH_S];
    default:
        return NULL;
    }
}

// 소수점을 포함한 FPS를 문자열 버퍼에 기록 후 길이 반환
static uint32_t Overlay_FormatFps(float fps, char *out) {
    uint32_t scaled, whole, frac, index = 0U;
    if (!(fps > 0.0f))
        fps = 0.0f; // 음수, NaN 필터링

    scaled = (uint32_t)((fps * 10.0f) + 0.5f);
    if (scaled > 9999U)
        scaled = 9999U; // 상한 999.9

    whole = scaled / 10U;
    frac  = scaled % 10U;

    out[index++] = 'F';
    out[index++] = 'P';
    out[index++] = 'S';
    out[index++] = ' ';
    if (whole >= 100U)
        out[index++] = (char) ('0' + (whole / 100U));
    if (whole >= 10U)
        out[index++] = (char) ('0' + ((whole / 10U) % 10U));
    out[index++] = (char)('0' + (whole % 10U));
    out[index++] = '.';
    out[index++] = (char)('0' + frac);
    out[index]   = '\0';

    return index;
}

// 1열(스캔라인) 내 지정 범위(가로)를 단일 색상으로 채움
static void Overlay_FillSpan(uint8_t *line, int32_t x, int32_t width, uint16_t color) {
    int32_t x_end = x + width;
    if (x < 0)
        x = 0;
    if (x_end > (int32_t) DISPLAY_WIDTH)
        x_end = (int32_t) DISPLAY_WIDTH;

    for (; x < x_end; x++) {
        line[x * 2]       = (uint8_t)(color >> 8);
        line[(x * 2) + 1] = (uint8_t)(color & 0xFFU);
    }
}

// 스캔라인 상에 사각형 그리기
static void Overlay_DrawRectLine(uint8_t *line, int32_t y, const OverlayRect_t *rect, int32_t grow, uint16_t color) {
    int32_t rect_y      = (int32_t)rect->y - grow;
    int32_t rect_height = (int32_t)rect->h + (2 * grow);
    if ((y < rect_y) || (y >= (rect_y + rect_height)))
        return;
    Overlay_FillSpan(line, (int32_t) rect->x - grow, (int32_t) rect->w + (2 * grow), color);
}

// 스캔라인 상에 폰트 조각 그리기
static void Overlay_DrawGlyphLine(uint8_t *line, int32_t y, int32_t glyph_x, int32_t glyph_y, const uint8_t *glyph, uint16_t color) {
    int32_t font_row;
    uint32_t column;
    if (y < glyph_y)
        return;
    font_row = (y - glyph_y) / (int32_t)OVERLAY_TEXT_SCALE;
    if (font_row >= (int32_t) OVERLAY_FONT_H)
        return;

    for (column = 0U; column < OVERLAY_FONT_W; column++) {
        if (((glyph[column] >> font_row) & 0x01U) != 0U) {
            Overlay_FillSpan(line, glyph_x + (int32_t) (column * OVERLAY_TEXT_SCALE), (int32_t) OVERLAY_TEXT_SCALE, color);
        }
    }
}

// 스캔라인 상에 전체 문자열 그리기
static void Overlay_DrawTextLine(uint8_t *line, int32_t y, int32_t text_x, int32_t text_y, const char *text, uint16_t color) {
    uint32_t index;
    for (index = 0U; text[index] != '\0'; index++) {
        const uint8_t *glyph = Overlay_GetGlyph(text[index]);
        if (glyph != NULL) {
            Overlay_DrawGlyphLine(line, y, text_x + (int32_t) (index * OVERLAY_CHAR_ADVANCE), text_y, glyph, color);
        }
    }
}

// 1프레임 렌더링 시작 전, 상태 및 FPS 스냅샷 생성
static void Overlay_BeginFrame(void) {
    uint8_t state = Motor_GetTrackState();
    float fps = Vision_GetCurrentFPS();
    uint32_t length;
    int32_t width;

    switch (state) {
    case MACHINE_STATE_TRACKING:
        overlay_cross_color = OVERLAY_COLOR_TRACKING;
        break;
    case MACHINE_STATE_LOCKON:
        overlay_cross_color = OVERLAY_COLOR_LOCKON;
        break;
    case MACHINE_STATE_MANUAL:
        overlay_cross_color = OVERLAY_COLOR_MANUAL;
        break;
    case MACHINE_STATE_IDLE:
    default:
        overlay_cross_color = OVERLAY_COLOR_IDLE;
        break;
    }

    length = Overlay_FormatFps(fps, overlay_text);
    width = (int32_t)(length * OVERLAY_CHAR_ADVANCE) - (int32_t)OVERLAY_CHAR_GAP;
    overlay_text_x = (int32_t) DISPLAY_WIDTH - OVERLAY_TEXT_MARGIN_X - width; // 우측 정렬
}

// 오버레이 UI 덮어쓰기 로직
static void Overlay_RenderLine(uint8_t *line, uint16_t y) {
    int32_t line_y = (int32_t)y;
    uint32_t index;

    // 렌더링 범위 밖 스캔라인은 무시하여 리소스 절약
    if (((line_y < OVERLAY_CROSS_TOP) || (line_y > OVERLAY_CROSS_BOTTOM)) &&
        ((line_y < OVERLAY_TEXT_TOP) || (line_y > OVERLAY_TEXT_BOTTOM))) {
        return;
    }

    // 외곽선 렌더링 후 안쪽 십자선 채우기
    for (index = 0U; index < OVERLAY_CROSS_RECT_COUNT; index++) {
        Overlay_DrawRectLine(line, line_y, &overlay_cross_rects[index], 1, OVERLAY_COLOR_OUTLINE);
    }
    for (index = 0U; index < OVERLAY_CROSS_RECT_COUNT; index++) {
        Overlay_DrawRectLine(line, line_y, &overlay_cross_rects[index], 0, overlay_cross_color);
    }

    // 텍스트 그림자 렌더링 후 본문 채우기
    Overlay_DrawTextLine(line, line_y, overlay_text_x + 1, OVERLAY_TEXT_MARGIN_Y + 1, overlay_text, OVERLAY_COLOR_OUTLINE);
    Overlay_DrawTextLine(line, line_y, overlay_text_x, OVERLAY_TEXT_MARGIN_Y, overlay_text, OVERLAY_COLOR_TEXT);
}

// ==============================================================================
// 외부 공개 API 함수
// ==============================================================================

void ActivateOverlayWidget(bool enable) {
    overlay_active = enable;
}

bool IsOverlayWidgetActive(void) {
    return overlay_active;
}

HAL_StatusTypeDef Display_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t data[15];

    HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_SET);
    Display_HardwareReset();

    // Software Reset
    status = Display_WriteCommand(ILI9341_CMD_SWRESET);
    if (status != HAL_OK)
        return status;
    HAL_Delay(150U);

    // Power control 등 초기 설정 스퀀스 생략 (기존 코드 유지)
    data[0] = 0x00U;
    data[1] = 0xC1U;
    data[2] = 0x30U;
    Display_WriteCommandData(0xCFU, data, 3U);
    data[0] = 0x64U;
    data[1] = 0x03U;
    data[2] = 0x12U;
    data[3] = 0x81U;
    Display_WriteCommandData(0xEDU, data, 4U);
    data[0] = 0x85U;
    data[1] = 0x00U;
    data[2] = 0x78U;
    Display_WriteCommandData(0xE8U, data, 3U);
    data[0] = 0x39U;
    data[1] = 0x2CU;
    data[2] = 0x00U;
    data[3] = 0x34U;
    data[4] = 0x02U;
    Display_WriteCommandData(0xCBU, data, 5U);
    data[0] = 0x20U;
    Display_WriteCommandData(0xF7U, data, 1U);
    data[0] = 0x00U;
    data[1] = 0x00U;
    Display_WriteCommandData(0xEAU, data, 2U);
    data[0] = 0x23U;
    Display_WriteCommandData(0xC0U, data, 1U);
    data[0] = 0x10U;
    Display_WriteCommandData(0xC1U, data, 1U);
    data[0] = 0x3EU;
    data[1] = 0x28U;
    Display_WriteCommandData(0xC5U, data, 2U);
    data[0] = 0x86U;
    Display_WriteCommandData(0xC7U, data, 1U);
    data[0] = DISPLAY_MADCTL_VALUE;
    Display_WriteCommandData(ILI9341_CMD_MADCTL, data, 1U);
    data[0] = 0x55U; // Pixel Format 16bit RGB565
    Display_WriteCommandData(ILI9341_CMD_PIXFMT, data, 1U);
    data[0] = 0x00U;
    data[1] = 0x18U;
    Display_WriteCommandData(0xB1U, data, 2U);
    data[0] = 0x08U;
    data[1] = 0x82U;
    data[2] = 0x27U;
    Display_WriteCommandData(0xB6U, data, 3U);
    data[0] = 0x00U;
    Display_WriteCommandData(0xF2U, data, 1U);
    data[0] = 0x01U;
    Display_WriteCommandData(0x26U, data, 1U);

    // Positive Gamma Correction
    data[0] = 0x0FU;
    data[1] = 0x31U;
    data[2] = 0x2BU;
    data[3] = 0x0CU;
    data[4] = 0x0EU;
    data[5] = 0x08U;
    data[6] = 0x4EU;
    data[7] = 0xF1U;
    data[8] = 0x37U;
    data[9] = 0x07U;
    data[10] = 0x10U;
    data[11] = 0x03U;
    data[12] = 0x0EU;
    data[13] = 0x09U;
    data[14] = 0x00U;
    Display_WriteCommandData(0xE0U, data, 15U);

    // Negative Gamma Correction
    data[0] = 0x00U;
    data[1] = 0x0EU;
    data[2] = 0x14U;
    data[3] = 0x03U;
    data[4] = 0x11U;
    data[5] = 0x07U;
    data[6] = 0x31U;
    data[7] = 0xC1U;
    data[8] = 0x48U;
    data[9] = 0x08U;
    data[10] = 0x0FU;
    data[11] = 0x0CU;
    data[12] = 0x31U;
    data[13] = 0x36U;
    data[14] = 0x0FU;
    Display_WriteCommandData(0xE1U, data, 15U);

    Display_WriteCommand(ILI9341_CMD_SLPOUT);
    HAL_Delay(120U);
    Display_WriteCommand(ILI9341_CMD_DISPON);
    HAL_Delay(20U);

    return HAL_OK;
}

// 1프레임 이미지를 LCD로 SPI DMA 전송 (RGB565 모드 최적화 + 핑퐁 버퍼 + LUT 스케일링 적용)
HAL_StatusTypeDef Display_UpdateImage(const uint16_t *image, uint16_t width, uint16_t height) {
    HAL_StatusTypeDef status;
    uint16_t dst_x, dst_y;
    uint32_t src_index;
    uint8_t buf_idx = 0;

    if (image == NULL || width == 0U || height == 0U)
        return HAL_ERROR;
    if ((width > DISPLAY_WIDTH) || (height > DISPLAY_HEIGHT))
        return HAL_ERROR;

    // 원본 데이터가 최신 상태가 되도록 캐시 무효화
    SCB_InvalidateDCache_by_Addr((uint32_t*) image, width * height * 2);

    status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U, DISPLAY_HEIGHT - 1U);
    if (status != HAL_OK)
        return status;

    DISPLAY_CS_LOW();
    DISPLAY_DC_DATA();

    // 오버레이 정보 스냅샷
    if (overlay_active)
        Overlay_BeginFrame();

    // ==============================================================================
    // LUT(룩업 테이블) 생성 및 업데이트 로직 (해상도가 바뀔 때만 1회 연산)
    // ==============================================================================
    if ((width != last_width) || (height != last_height)) {
        // Y 좌표 매핑 및 width 곱셈 미리 계산 (src_y * width)
        for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
            uint32_t src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;
#if (DISPLAY_CAMERA_FLIP_180 == 1)
            src_y = (uint32_t) (height - 1U) - src_y;
#endif
            lut_y[dst_y] = src_y * width;
        }

        // X 좌표 매핑 계산
        for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
            uint32_t src_x = (uint32_t) (width - 1U) - (((uint32_t) dst_x * width) / DISPLAY_WIDTH);
#if (DISPLAY_CAMERA_FLIP_180 == 1)
            src_x = (uint32_t) (width - 1U) - src_x;
#endif
            lut_x[dst_x] = src_x;
        }

        last_width = width;
        last_height = height;
    }

    display_dma_completed = 1;

    for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
        // 루프 밖에서 현재 줄의 Y 오프셋을 한 번만 가져옴
        uint32_t src_y_offset = lut_y[dst_y];

        for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
            // 나눗셈/곱셈 없이 더하기 연산 1번으로 인덱스 도출
            src_index = src_y_offset + lut_x[dst_x];
            uint16_t pixel = image[src_index];

            display_tx_buffer[buf_idx][dst_x * 2U] = (uint8_t) (pixel >> 8);
            display_tx_buffer[buf_idx][(dst_x * 2U) + 1U] = (uint8_t) (pixel & 0xFFU);
        }

        if (overlay_active)
            Overlay_RenderLine(display_tx_buffer[buf_idx], dst_y);

        SCB_CleanDCache_by_Addr((uint32_t*) display_tx_buffer[buf_idx], DISPLAY_WIDTH * 2U);

        while (display_dma_completed == 0) {
        }
        display_dma_completed = 0;

        if (HAL_SPI_Transmit_DMA(&hspi2, display_tx_buffer[buf_idx], DISPLAY_WIDTH * 2U) != HAL_OK) {
            DISPLAY_CS_HIGH();
            return HAL_ERROR;
        }

        buf_idx ^= 1;
    }

    while (display_dma_completed == 0) {
    }
    DISPLAY_CS_HIGH();

    return HAL_OK;
}

// SPI DMA 전송 완료 콜백 함수
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI2) {
        display_dma_completed = 1;
    }
}
// YUV 모드
//HAL_StatusTypeDef Display_UpdateImage(const uint16_t *image, uint16_t width,
//      uint16_t height) {
//  HAL_StatusTypeDef status;
//  uint16_t dst_x;
//  uint16_t dst_y;
//  uint32_t src_x;
//  uint32_t src_y;
//  uint32_t src_index;
//
//  if (image == NULL || width == 0U || height == 0U) {
//      return HAL_ERROR;
//  }
//
//  if ((width > DISPLAY_WIDTH) || (height > DISPLAY_HEIGHT)) {
//      return HAL_ERROR;
//  }
//
//  // 1. STM32H7 캐시 동기화 (카메라로 받은 원본 이미지 메모리 무효화)
//  SCB_InvalidateDCache_by_Addr((uint32_t*) image, width * height * 2);
//
//  status = Display_SetAddressWindow(0U, 0U, DISPLAY_WIDTH - 1U,
//  DISPLAY_HEIGHT - 1U);
//  if (status != HAL_OK) {
//      return status;
//  }
//
//  DISPLAY_CS_LOW();
//  DISPLAY_DC_DATA();
//
//  uint8_t *p_bytes = (uint8_t*) image;
//
//  for (dst_y = 0U; dst_y < DISPLAY_HEIGHT; dst_y++) {
//      src_y = ((uint32_t) dst_y * height) / DISPLAY_HEIGHT;
//
//      for (dst_x = 0U; dst_x < DISPLAY_WIDTH; dst_x++) {
//          src_x = ((uint32_t) dst_x * width) / DISPLAY_WIDTH;
//          src_index = (src_y * width) + src_x;
//
//          // current_mode가 1(RGB)일 때
//          if (current_mode == 1) {
//              uint16_t pixel = image[src_index];
//              display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
//              display_tx_buffer[(dst_x * 2U) + 1U] =
//                      (uint8_t) (pixel & 0xFFU);
//          }
//          // current_mode가 0(Grayscale)일 때
//          else {
//              uint8_t *p_bytes = (uint8_t*) image;
//              uint8_t y_val = p_bytes[src_index * 2U + 1U];
//
//              uint16_t r = (y_val >> 3) & 0x1FU;
//              uint16_t g = (y_val >> 2) & 0x3FU;
//              uint16_t b = (y_val >> 3) & 0x1FU;
//              uint16_t pixel = (r << 11) | (g << 5) | b;
//
//              display_tx_buffer[dst_x * 2U] = (uint8_t) (pixel >> 8);
//              display_tx_buffer[(dst_x * 2U) + 1U] =
//                      (uint8_t) (pixel & 0xFFU);
//          }
//      }
//
//      // 2. 전송할 텍스처 버퍼 캐시 클린 (메모리에 확실히 써지도록 함)
//      SCB_CleanDCache_by_Addr((uint32_t*) display_tx_buffer,
//      DISPLAY_WIDTH * 2U);
//
//      // 3. SPI DMA 전송 시작
//      display_dma_completed = 0;
//      if (HAL_SPI_Transmit_DMA(&hspi2, display_tx_buffer, DISPLAY_WIDTH * 2U)
//              != HAL_OK) {
//          DISPLAY_CS_HIGH();
//          return HAL_ERROR;
//      }
//
//      // 4. DMA 전송이 완료될 때까지 대기
//      while (display_dma_completed == 0) {
//          // 대기 중 칩셋 부하를 줄이려면 __WFI(); 삽입 가능
//      }
//  }
//
//  DISPLAY_CS_HIGH();
//  return HAL_OK;
//}
