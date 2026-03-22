#include "Com_IMU.h"
#include <math.h>

#define IMU_RAD2DEG 57.2957795f
//保存零偏角度
static IMU_AxisFloat_t g_gyro_bias_dps = {0.0f, 0.0f, 0.0f};
//保存姿态角
static IMU_Attitude_t g_attitude = {0.0f, 0.0f, 0.0f};

//读到三轴原始数据
static HAL_StatusTypeDef IMU_ReadRaw3Axis(uint8_t start_reg, IMU_AxisRaw_t *out)
{
	uint8_t data[6];
	HAL_StatusTypeDef ret;

	if (out == NULL)
	{
		return HAL_ERROR;
	}

	ret = Com_IMU_ReadRegs(start_reg, data, 6U);
	if (ret != HAL_OK)
	{
		return ret;
	}
	//已配置为Little Endian,低字节在前，高字节在后
	out->x = (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
	out->y = (int16_t)(((uint16_t)data[3] << 8U) | data[2]);
	out->z = (int16_t)(((uint16_t)data[5] << 8U) | data[4]);

	return HAL_OK;
}
//写寄存器

HAL_StatusTypeDef Com_IMU_WriteReg(uint8_t reg, uint8_t value)
{
	uint8_t tx[2];

	tx[0] = (uint8_t)(reg & (uint8_t)(~IMU_SPI_READ_MASK));//寄存器地址(最高位为0表示写操作)
	tx[1] = value;//要写入的数据

	IMU_CS_LOW();
	if (HAL_SPI_Transmit(&hspi1, tx, 2U, IMU_SPI_TIMEOUT_MS) != HAL_OK)
	{
		IMU_CS_HIGH();
		return HAL_ERROR;
	}
	IMU_CS_HIGH();

	return HAL_OK;
}

HAL_StatusTypeDef Com_IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint8_t addr;

	if ((buf == NULL) || (len == 0U))
	{
		return HAL_ERROR;
	}

	addr = (uint8_t)(reg | IMU_SPI_READ_MASK);//寄存器地址(最高位为1表示读操作)

	IMU_CS_LOW();

	if (HAL_SPI_Transmit(&hspi1, &addr, 1U, IMU_SPI_TIMEOUT_MS) != HAL_OK)
	{
		IMU_CS_HIGH();
		return HAL_ERROR;
	}

	if (HAL_SPI_Receive(&hspi1, buf, len, IMU_SPI_TIMEOUT_MS) != HAL_OK)
	{
		IMU_CS_HIGH();
		return HAL_ERROR;
	}

	IMU_CS_HIGH();
	return HAL_OK;
}
//读单字节寄存器
HAL_StatusTypeDef Com_IMU_ReadReg(uint8_t reg, uint8_t *value)
{
	return Com_IMU_ReadRegs(reg, value, 1U);
}
//读WHO_AM_I寄存器验证通信
HAL_StatusTypeDef Com_IMU_ReadWhoAmI(uint8_t *whoami)
{
	return Com_IMU_ReadReg(ICM45686_REG_WHO_AM_I, whoami);
}

HAL_StatusTypeDef Com_IMU_Init(void)
{
	uint8_t whoami = 0U;
	uint8_t pwr_cfg;
	uint8_t gyro_cfg;
	uint8_t accel_cfg;
	IMU_CS_HIGH();
	vTaskDelay(pdMS_TO_TICKS(2));
	if (Com_IMU_ReadWhoAmI(&whoami) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (whoami != ICM45686_WHO_AM_I_DEFAULT)
	{
		return HAL_ERROR;
	}
	//将陀螺仪和加速度计都配置为LN模式（低噪声模式），即陀螺仪和加速度计都持续测量并输出数据，适合需要高性能和低噪声的应用场景。LN模式下，陀螺仪和加速度计的功耗较高，但可以获得更好的测量精度和更低的噪声水平。
	pwr_cfg = (uint8_t)(ICM45686_GYRO_MODE_LN | ICM45686_ACCEL_MODE_LN);
	if (Com_IMU_WriteReg(ICM45686_REG_PWR_MGMT0, pwr_cfg) != HAL_OK)
	{
		return HAL_ERROR;
	}
	//配置量程，陀螺仪：2000dps，200Hz ODR；加速度计：±2g，200Hz ODR
	gyro_cfg = (uint8_t)(ICM45686_GYRO_FS_2000DPS | ICM45686_GYRO_ODR_200HZ);
	if (Com_IMU_WriteReg(ICM45686_REG_GYRO_CONFIG0, gyro_cfg) != HAL_OK)
	{
		return HAL_ERROR;
	}
	accel_cfg = (uint8_t)(ICM45686_ACCEL_FS_2G | ICM45686_ACCEL_ODR_200HZ);
	if (Com_IMU_WriteReg(ICM45686_REG_ACCEL_CONFIG0, accel_cfg) != HAL_OK)
	{
		return HAL_ERROR;
	}
	//等待稳定
	vTaskDelay(pdMS_TO_TICKS(40));
	return HAL_OK;
}
//从陀螺仪起始寄存器直接读6个字节
HAL_StatusTypeDef Com_IMU_ReadGyroRaw(IMU_AxisRaw_t *gyro_raw)
{
	return IMU_ReadRaw3Axis(ICM45686_REG_GYRO_DATA_X1, gyro_raw);
}
//从加速度计起始寄存器直接读6个字节
HAL_StatusTypeDef Com_IMU_ReadAccelRaw(IMU_AxisRaw_t *accel_raw)
{
	return IMU_ReadRaw3Axis(ICM45686_REG_ACCEL_DATA_X1, accel_raw);
}
//原始数值转变为角速度->量程/灵敏度
void Com_IMU_GyroRawToDps(const IMU_AxisRaw_t *raw, IMU_AxisFloat_t *dps, float lsb_per_dps)
{
	if ((raw == NULL) || (dps == NULL) || (lsb_per_dps <= 0.0f))
	{
		return;
	}

	dps->x = ((float)raw->x) / lsb_per_dps;
	dps->y = ((float)raw->y) / lsb_per_dps;
	dps->z = ((float)raw->z) / lsb_per_dps;
}
//度每秒转为弧度每秒
void Com_IMU_GyroDpsToRad(const IMU_AxisFloat_t *dps, IMU_AxisFloat_t *rad_s)
{
	const float deg2rad = 0.0174532925f;

	if ((dps == NULL) || (rad_s == NULL))
	{
		return;
	}

	rad_s->x = dps->x * deg2rad;
	rad_s->y = dps->y * deg2rad;
	rad_s->z = dps->z * deg2rad;
}
//设置零偏
void Com_IMU_SetGyroBiasDps(const IMU_AxisFloat_t *bias_dps)
{
	if (bias_dps == NULL)
	{
		return;
	}

	g_gyro_bias_dps = *bias_dps;
}

HAL_StatusTypeDef Com_IMU_CalibrateGyroBias(uint16_t samples, uint16_t interval_ms, float gyro_lsb_per_dps, IMU_AxisFloat_t *bias_out)
{
	uint32_t i;
	IMU_AxisRaw_t gyro_raw;
	IMU_AxisFloat_t gyro_dps;
	IMU_AxisFloat_t sum = {0.0f, 0.0f, 0.0f};

	if ((samples == 0U) || (gyro_lsb_per_dps <= 0.0f))
	{
		return HAL_ERROR;
	}

	for (i = 0U; i < samples; i++)
	{
		if (Com_IMU_ReadGyroRaw(&gyro_raw) != HAL_OK)
		{
			return HAL_ERROR;
		}

		Com_IMU_GyroRawToDps(&gyro_raw, &gyro_dps, gyro_lsb_per_dps);
		sum.x += gyro_dps.x;
		sum.y += gyro_dps.y;
		sum.z += gyro_dps.z;

		if (interval_ms > 0U)
		{
			vTaskDelay(pdMS_TO_TICKS(interval_ms));
		}
	}

	g_gyro_bias_dps.x = sum.x / (float)samples;
	g_gyro_bias_dps.y = sum.y / (float)samples;
	g_gyro_bias_dps.z = sum.z / (float)samples;

	if (bias_out != NULL)
	{
		*bias_out = g_gyro_bias_dps;
	}

	return HAL_OK;
}
//姿态更新
HAL_StatusTypeDef Com_IMU_UpdateAttitude(float dt_s, float gyro_lsb_per_dps, float accel_lsb_per_g, float alpha, IMU_Attitude_t *att_out)
{
	IMU_AxisRaw_t gyro_raw;
	IMU_AxisRaw_t accel_raw;
	IMU_AxisFloat_t gyro_dps;
	float ax_g;
	float ay_g;
	float az_g;
	float accel_roll_deg;
	float accel_pitch_deg;

	if ((dt_s <= 0.0f) || (gyro_lsb_per_dps <= 0.0f) || (accel_lsb_per_g <= 0.0f))
	{
		return HAL_ERROR;
	}

	if (alpha < 0.0f)
	{
		alpha = 0.0f;
	}
	else if (alpha > 1.0f)
	{
		alpha = 1.0f;
	}

	if (Com_IMU_ReadGyroRaw(&gyro_raw) != HAL_OK)
	{
		return HAL_ERROR;
	}
	if (Com_IMU_ReadAccelRaw(&accel_raw) != HAL_OK)
	{
		return HAL_ERROR;
	}

	Com_IMU_GyroRawToDps(&gyro_raw, &gyro_dps, gyro_lsb_per_dps);
	gyro_dps.x -= g_gyro_bias_dps.x;
	gyro_dps.y -= g_gyro_bias_dps.y;
	gyro_dps.z -= g_gyro_bias_dps.z;

	ax_g = ((float)accel_raw.x) / accel_lsb_per_g;
	ay_g = ((float)accel_raw.y) / accel_lsb_per_g;
	az_g = ((float)accel_raw.z) / accel_lsb_per_g;

	accel_roll_deg = atan2f(ay_g, az_g) * IMU_RAD2DEG;
	accel_pitch_deg = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * IMU_RAD2DEG;

	g_attitude.roll_deg += gyro_dps.x * dt_s;
	g_attitude.pitch_deg += gyro_dps.y * dt_s;
	g_attitude.yaw_deg += gyro_dps.z * dt_s;

	g_attitude.roll_deg = alpha * g_attitude.roll_deg + (1.0f - alpha) * accel_roll_deg;
	g_attitude.pitch_deg = alpha * g_attitude.pitch_deg + (1.0f - alpha) * accel_pitch_deg;

	if (att_out != NULL)
	{
		*att_out = g_attitude;
	}

	return HAL_OK;
}

void Com_IMU_GetAttitude(IMU_Attitude_t *att_out)
{
	if (att_out == NULL)
	{
		return;
	}

	*att_out = g_attitude;
}