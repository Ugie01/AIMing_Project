/*
 * ov2640_io.c
 *
 *  Created on: 2026. 7. 26.
 *      Author: KCCISTC
 */




#include "ov2640.h"
#include "i2c.h" // hi2c1 사용

// OV2640 I2C 7-bit 주소 (0x30) -> 8-bit Shift (0x60)
#define OV2640_I2C_ADDRESS  (0x30 << 1)

void CAMERA_IO_Init(void) {
	// CubeMX MX_I2C1_Init()에서 이미 하드웨어를 초기화하므로 비워두어도 됩니다.
}

void CAMERA_IO_Write(uint8_t addr, uint8_t reg, uint8_t value) {
	// addr 인자는 BSP 드라이버에서 넘어오지만 safety 차원에서 8bit 변환 주소 사용
	HAL_I2C_Mem_Write(&hi2c1, OV2640_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 100);
}

uint8_t CAMERA_IO_Read(uint8_t addr, uint8_t reg) {
	uint8_t value = 0;
	HAL_I2C_Mem_Read(&hi2c1, OV2640_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 100);
	return value;
}

void CAMERA_Delay(uint32_t delay) {
	HAL_Delay(delay);
}
