#ifndef __COM_IMU_H__
#define __COM_IMU_H__

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "spi.h"
#include "cmsis_os.h"

#define IMU_SPI_TIMEOUT_MS  100U

/* SPI read command bit: set MSB for read. */
#define IMU_SPI_READ_MASK   0x80U

/* Your hardware uses PA4 as software chip-select. */
#define IMU_CS_LOW()   HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define IMU_CS_HIGH()  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

/* ICM-45686 User Bank 0 register map */
#define ICM45686_REG_ACCEL_DATA_X1       0x00U
#define ICM45686_REG_ACCEL_DATA_X0       0x01U
#define ICM45686_REG_ACCEL_DATA_Y1       0x02U
#define ICM45686_REG_ACCEL_DATA_Y0       0x03U
#define ICM45686_REG_ACCEL_DATA_Z1       0x04U
#define ICM45686_REG_ACCEL_DATA_Z0       0x05U

#define ICM45686_REG_GYRO_DATA_X1        0x06U
#define ICM45686_REG_GYRO_DATA_X0        0x07U
#define ICM45686_REG_GYRO_DATA_Y1        0x08U
#define ICM45686_REG_GYRO_DATA_Y0        0x09U
#define ICM45686_REG_GYRO_DATA_Z1        0x0AU
#define ICM45686_REG_GYRO_DATA_Z0        0x0BU

#define ICM45686_REG_TEMP_DATA1          0x0CU
#define ICM45686_REG_TEMP_DATA0          0x0DU

#define ICM45686_REG_PWR_MGMT0           0x10U
#define ICM45686_REG_INT1_STATUS0        0x19U
#define ICM45686_REG_ACCEL_CONFIG0       0x1BU
#define ICM45686_REG_GYRO_CONFIG0        0x1CU

#define ICM45686_REG_WHO_AM_I            0x72U

#define ICM45686_WHO_AM_I_DEFAULT        0xE9U

/* PWR_MGMT0[3:2] GYRO_MODE, [1:0] ACCEL_MODE */
#define ICM45686_GYRO_MODE_OFF           (0x0U << 2)
#define ICM45686_GYRO_MODE_STANDBY       (0x1U << 2)
#define ICM45686_GYRO_MODE_LP            (0x2U << 2)
#define ICM45686_GYRO_MODE_LN            (0x3U << 2)

#define ICM45686_ACCEL_MODE_OFF          (0x0U << 0)
#define ICM45686_ACCEL_MODE_OFF_ALT      (0x1U << 0)
#define ICM45686_ACCEL_MODE_LP           (0x2U << 0)
#define ICM45686_ACCEL_MODE_LN           (0x3U << 0)

/* GYRO_CONFIG0[7:4] FS, [3:0] ODR */
#define ICM45686_GYRO_FS_4000DPS         (0x0U << 4)
#define ICM45686_GYRO_FS_2000DPS         (0x1U << 4)
#define ICM45686_GYRO_FS_1000DPS         (0x2U << 4)
#define ICM45686_GYRO_FS_500DPS          (0x3U << 4)
#define ICM45686_GYRO_FS_250DPS          (0x4U << 4)
#define ICM45686_GYRO_FS_125DPS          (0x5U << 4)
#define ICM45686_GYRO_FS_62DPS           (0x6U << 4)
#define ICM45686_GYRO_FS_31DPS           (0x7U << 4)
#define ICM45686_GYRO_FS_15DPS           (0x8U << 4)

#define ICM45686_GYRO_ODR_6400HZ         0x03U
#define ICM45686_GYRO_ODR_3200HZ         0x04U
#define ICM45686_GYRO_ODR_1600HZ         0x05U
#define ICM45686_GYRO_ODR_800HZ          0x06U
#define ICM45686_GYRO_ODR_400HZ          0x07U
#define ICM45686_GYRO_ODR_200HZ          0x08U
#define ICM45686_GYRO_ODR_100HZ          0x09U
#define ICM45686_GYRO_ODR_50HZ           0x0AU
#define ICM45686_GYRO_ODR_25HZ           0x0BU
#define ICM45686_GYRO_ODR_12P5HZ         0x0CU
#define ICM45686_GYRO_ODR_6P25HZ         0x0DU
#define ICM45686_GYRO_ODR_3P125HZ        0x0EU
#define ICM45686_GYRO_ODR_1P5625HZ       0x0FU

/* ACCEL_CONFIG0[6:4] FS, [3:0] ODR */
#define ICM45686_ACCEL_FS_32G            (0x0U << 4)
#define ICM45686_ACCEL_FS_16G            (0x1U << 4)
#define ICM45686_ACCEL_FS_8G             (0x2U << 4)
#define ICM45686_ACCEL_FS_4G             (0x3U << 4)
#define ICM45686_ACCEL_FS_2G             (0x4U << 4)

#define ICM45686_ACCEL_ODR_6400HZ        0x03U
#define ICM45686_ACCEL_ODR_3200HZ        0x04U
#define ICM45686_ACCEL_ODR_1600HZ        0x05U
#define ICM45686_ACCEL_ODR_800HZ         0x06U
#define ICM45686_ACCEL_ODR_400HZ         0x07U
#define ICM45686_ACCEL_ODR_200HZ         0x08U
#define ICM45686_ACCEL_ODR_100HZ         0x09U
#define ICM45686_ACCEL_ODR_50HZ          0x0AU
#define ICM45686_ACCEL_ODR_25HZ          0x0BU
#define ICM45686_ACCEL_ODR_12P5HZ        0x0CU
#define ICM45686_ACCEL_ODR_6P25HZ        0x0DU
#define ICM45686_ACCEL_ODR_3P125HZ       0x0EU
#define ICM45686_ACCEL_ODR_1P5625HZ      0x0FU

/* Default conversion factors used by this driver */
#define ICM45686_GYRO_LSB_PER_DPS_2000   16.4f
#define ICM45686_ACCEL_LSB_PER_G_2G      16384.0f

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
} IMU_AxisRaw_t;

typedef struct
{
	float x;
	float y;
	float z;
} IMU_AxisFloat_t;

typedef struct
{
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
} IMU_Attitude_t;

HAL_StatusTypeDef Com_IMU_Init(void);
HAL_StatusTypeDef Com_IMU_ReadWhoAmI(uint8_t *whoami);

HAL_StatusTypeDef Com_IMU_ReadGyroRaw(IMU_AxisRaw_t *gyro_raw);
HAL_StatusTypeDef Com_IMU_ReadAccelRaw(IMU_AxisRaw_t *accel_raw);

/* Convert raw gyro to dps/rad_s according to configured full-scale (example default: 2000dps => 16.4 LSB/dps). */
void Com_IMU_GyroRawToDps(const IMU_AxisRaw_t *raw, IMU_AxisFloat_t *dps, float lsb_per_dps);
void Com_IMU_GyroDpsToRad(const IMU_AxisFloat_t *dps, IMU_AxisFloat_t *rad_s);

/* Bias and attitude fusion helpers. */
void Com_IMU_SetGyroBiasDps(const IMU_AxisFloat_t *bias_dps);
HAL_StatusTypeDef Com_IMU_CalibrateGyroBias(uint16_t samples, uint16_t interval_ms, float gyro_lsb_per_dps, IMU_AxisFloat_t *bias_out);
HAL_StatusTypeDef Com_IMU_UpdateAttitude(float dt_s, float gyro_lsb_per_dps, float accel_lsb_per_g, float alpha, IMU_Attitude_t *att_out);
void Com_IMU_GetAttitude(IMU_Attitude_t *att_out);

/* Low-level access in case user wants custom register operations. */
HAL_StatusTypeDef Com_IMU_WriteReg(uint8_t reg, uint8_t value);
HAL_StatusTypeDef Com_IMU_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef Com_IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);

#endif