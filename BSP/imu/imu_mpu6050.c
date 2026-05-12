#include "imu_mpu6050.h"

#include "i2c.h"

#define MPU6050_ADDR_0              (0x68U << 1)
#define MPU6050_ADDR_1              (0x69U << 1)
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_WHO_AM_I        0x75U
#define MPU6050_WHO_AM_I_MPU6050    0x68U
#define MPU6050_WHO_AM_I_MPU6500    0x70U

#define MPU6050_I2C_TIMEOUT_MS      2U
#define MPU6050_I2C_ASYNC_TIMEOUT_MS 25U
#define MPU6050_ACCEL_LSB_PER_G     16384.0f
#define MPU6050_GYRO_LSB_PER_DPS    131.0f

static imu_mpu6050_state_t imu_state;
static uint16_t mpu6050_addr;
static volatile uint8_t imu_busy;
static volatile uint8_t imu_enabled;
static volatile uint8_t imu_rx_done;
static volatile uint8_t imu_rx_error;
static uint8_t imu_rx_buffer[14];
static uint32_t imu_rx_start_ms;
static volatile uint32_t imu_rx_done_ms;

static void ImuMpu6050_SetError(uint8_t error)
{
  imu_state.last_error = error;
  if (error != IMU_MPU6050_ERROR_NONE)
  {
    imu_state.error_count++;
  }
}

static uint8_t ImuMpu6050_TryLock(void)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t locked = 0U;

  __disable_irq();
  if (imu_busy == 0U)
  {
    imu_busy = 1U;
    locked = 1U;
  }
  __set_PRIMASK(primask);
  return locked;
}

static void ImuMpu6050_Unlock(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  imu_busy = 0U;
  __set_PRIMASK(primask);
}

static int16_t ImuMpu6050_ReadI16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void ImuMpu6050_ParseSample(const uint8_t *data)
{
  imu_state.accel_x = ImuMpu6050_ReadI16(&data[0]);
  imu_state.accel_y = ImuMpu6050_ReadI16(&data[2]);
  imu_state.accel_z = ImuMpu6050_ReadI16(&data[4]);
  imu_state.temperature_raw = ImuMpu6050_ReadI16(&data[6]);
  imu_state.gyro_x = ImuMpu6050_ReadI16(&data[8]);
  imu_state.gyro_y = ImuMpu6050_ReadI16(&data[10]);
  imu_state.gyro_z = ImuMpu6050_ReadI16(&data[12]);

  imu_state.accel_g[0] = (float)imu_state.accel_x / MPU6050_ACCEL_LSB_PER_G;
  imu_state.accel_g[1] = (float)imu_state.accel_y / MPU6050_ACCEL_LSB_PER_G;
  imu_state.accel_g[2] = (float)imu_state.accel_z / MPU6050_ACCEL_LSB_PER_G;
  imu_state.temperature_c = ((float)imu_state.temperature_raw / 340.0f) + 36.53f;
  imu_state.gyro_dps[0] = (float)imu_state.gyro_x / MPU6050_GYRO_LSB_PER_DPS;
  imu_state.gyro_dps[1] = (float)imu_state.gyro_y / MPU6050_GYRO_LSB_PER_DPS;
  imu_state.gyro_dps[2] = (float)imu_state.gyro_z / MPU6050_GYRO_LSB_PER_DPS;
  imu_state.last_update_ms = HAL_GetTick();
  imu_state.last_read_ms = imu_rx_done_ms - imu_rx_start_ms;
  imu_state.online = 1U;
  imu_state.transfer_state = IMU_MPU6050_TRANSFER_IDLE;
  imu_state.last_i2c_error = 0U;
  ImuMpu6050_SetError(IMU_MPU6050_ERROR_NONE);
}

static uint8_t ImuMpu6050_IsSupportedWhoAmI(uint8_t who_am_i)
{
  return ((who_am_i == MPU6050_WHO_AM_I_MPU6050) ||
          (who_am_i == MPU6050_WHO_AM_I_MPU6500)) ? 1U : 0U;
}

static uint8_t ImuMpu6050_ReadReg(uint8_t reg, uint8_t *value)
{
  return (HAL_I2C_Mem_Read(&hi2c1,
                           mpu6050_addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           value,
                           1U,
                           MPU6050_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t ImuMpu6050_WriteReg(uint8_t reg, uint8_t value)
{
  return (HAL_I2C_Mem_Write(&hi2c1,
                            mpu6050_addr,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            &value,
                            1U,
                            MPU6050_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

static uint8_t ImuMpu6050_SelectAddress(void)
{
  imu_state.last_probe_ms = HAL_GetTick();

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR_0, 2U, MPU6050_I2C_TIMEOUT_MS) == HAL_OK)
  {
    mpu6050_addr = MPU6050_ADDR_0;
    imu_state.detected_addr = 0x68U;
    return 1U;
  }
  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR_1, 2U, MPU6050_I2C_TIMEOUT_MS) == HAL_OK)
  {
    mpu6050_addr = MPU6050_ADDR_1;
    imu_state.detected_addr = 0x69U;
    return 1U;
  }
  imu_state.detected_addr = 0U;
  ImuMpu6050_SetError(IMU_MPU6050_ERROR_NO_DEVICE);
  return 0U;
}

static uint8_t ImuMpu6050_Configure(void)
{
  uint8_t who_am_i = 0U;

  if (ImuMpu6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) == 0U)
  {
    imu_state.who_am_i = 0U;
    ImuMpu6050_SetError(IMU_MPU6050_ERROR_WHOAMI_READ_FAIL);
    return 0U;
  }
  imu_state.who_am_i = who_am_i;
  if (ImuMpu6050_IsSupportedWhoAmI(who_am_i) == 0U)
  {
    ImuMpu6050_SetError(IMU_MPU6050_ERROR_WHOAMI_MISMATCH);
    return 0U;
  }

  ImuMpu6050_SetError(IMU_MPU6050_ERROR_NONE);
  return 1U;
}

void ImuMpu6050_Init(void)
{
  imu_state = (imu_mpu6050_state_t){0};
  mpu6050_addr = MPU6050_ADDR_0;
  imu_busy = 0U;
  imu_enabled = 0U;
  imu_rx_done = 0U;
  imu_rx_error = 0U;
  imu_state.enabled = 0U;
  imu_state.transfer_state = IMU_MPU6050_TRANSFER_IDLE;
}

uint8_t ImuMpu6050_SetEnabled(uint8_t enabled)
{
  if (enabled == 0U)
  {
    imu_enabled = 0U;
    imu_state.enabled = 0U;
    return 1U;
  }

  imu_enabled = 1U;
  imu_state.enabled = 1U;
  return 1U;
}

uint8_t ImuMpu6050_IsEnabled(void)
{
  return imu_enabled;
}

uint8_t ImuMpu6050_ProbeNow(void)
{
  uint8_t who_am_i = 0U;
  uint8_t ok = 0U;

  if (ImuMpu6050_TryLock() == 0U)
  {
    return 0U;
  }

  if (ImuMpu6050_SelectAddress() != 0U &&
      ImuMpu6050_ReadReg(MPU6050_REG_WHO_AM_I, &who_am_i) != 0U)
  {
    imu_state.who_am_i = who_am_i;
    if (ImuMpu6050_IsSupportedWhoAmI(who_am_i) != 0U)
    {
      ImuMpu6050_SetError(IMU_MPU6050_ERROR_NONE);
      ok = 1U;
    }
    else
    {
      ImuMpu6050_SetError(IMU_MPU6050_ERROR_WHOAMI_MISMATCH);
    }
  }
  else
  {
    imu_state.who_am_i = 0U;
    ImuMpu6050_SetError(IMU_MPU6050_ERROR_WHOAMI_READ_FAIL);
  }

  imu_state.online = 0U;
  ImuMpu6050_Unlock();
  return ok;
}

uint8_t ImuMpu6050_ConfigNow(void)
{
  uint8_t ok = 0U;

  if (ImuMpu6050_TryLock() == 0U)
  {
    return 0U;
  }

  if (ImuMpu6050_SelectAddress() != 0U && ImuMpu6050_Configure() != 0U)
  {
    imu_state.online = 1U;
    ok = 1U;
  }
  else
  {
    imu_state.online = 0U;
  }

  ImuMpu6050_Unlock();
  return ok;
}

uint8_t ImuMpu6050_WriteRegTest(uint8_t reg, uint8_t value, uint8_t *readback, uint32_t *hal_error)
{
  uint8_t ok = 0U;
  uint8_t rb = 0U;

  if (ImuMpu6050_TryLock() == 0U)
  {
    if (readback != 0)
    {
      *readback = 0U;
    }
    if (hal_error != 0)
    {
      *hal_error = HAL_BUSY;
    }
    return 0U;
  }

  if (ImuMpu6050_SelectAddress() != 0U &&
      ImuMpu6050_WriteReg(reg, value) != 0U &&
      ImuMpu6050_ReadReg(reg, &rb) != 0U)
  {
    ok = 1U;
  }

  if (readback != 0)
  {
    *readback = rb;
  }
  if (hal_error != 0)
  {
    *hal_error = HAL_I2C_GetError(&hi2c1);
  }

  ImuMpu6050_Unlock();
  return ok;
}

uint8_t ImuMpu6050_RequestRead(void)
{
  HAL_StatusTypeDef status;

  if (ImuMpu6050_TryLock() == 0U)
  {
    return 0U;
  }

  if (imu_state.online == 0U)
  {
    ImuMpu6050_Unlock();
    return 0U;
  }

  imu_rx_done = 0U;
  imu_rx_error = 0U;
  imu_rx_start_ms = HAL_GetTick();
  imu_rx_done_ms = imu_rx_start_ms;
  imu_state.transfer_state = IMU_MPU6050_TRANSFER_READING;
  status = HAL_I2C_Mem_Read_IT(&hi2c1,
                               mpu6050_addr,
                               MPU6050_REG_ACCEL_XOUT_H,
                               I2C_MEMADD_SIZE_8BIT,
                               imu_rx_buffer,
                               sizeof(imu_rx_buffer));
  if (status != HAL_OK)
  {
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_ERROR;
    imu_state.last_i2c_error = HAL_I2C_GetError(&hi2c1);
    ImuMpu6050_SetError(IMU_MPU6050_ERROR_READ_FAIL);
    ImuMpu6050_Unlock();
    return 0U;
  }

  return 1U;
}

void ImuMpu6050_Update(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (imu_state.transfer_state == IMU_MPU6050_TRANSFER_READING)
  {
    if ((now_ms - imu_rx_start_ms) > MPU6050_I2C_ASYNC_TIMEOUT_MS)
    {
      (void)HAL_I2C_Master_Abort_IT(&hi2c1, mpu6050_addr);
      imu_state.transfer_state = IMU_MPU6050_TRANSFER_TIMEOUT;
      imu_state.last_i2c_error = HAL_I2C_GetError(&hi2c1);
      imu_state.last_read_ms = now_ms - imu_rx_start_ms;
      imu_state.online = 0U;
      ImuMpu6050_SetError(IMU_MPU6050_ERROR_READ_TIMEOUT);
      return;
    }
    return;
  }

  if (imu_state.transfer_state == IMU_MPU6050_TRANSFER_DONE)
  {
    imu_rx_done = 0U;
    ImuMpu6050_ParseSample(imu_rx_buffer);
    ImuMpu6050_Unlock();
    return;
  }

  if (imu_state.transfer_state == IMU_MPU6050_TRANSFER_ERROR)
  {
    imu_rx_error = 0U;
    imu_state.last_i2c_error = HAL_I2C_GetError(&hi2c1);
    imu_state.online = 0U;
    ImuMpu6050_SetError(IMU_MPU6050_ERROR_READ_FAIL);
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_IDLE;
    ImuMpu6050_Unlock();
    return;
  }

  if (imu_state.transfer_state == IMU_MPU6050_TRANSFER_TIMEOUT)
  {
    imu_rx_error = 0U;
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_IDLE;
    ImuMpu6050_Unlock();
    return;
  }

  if ((imu_enabled != 0U) && (imu_state.online != 0U))
  {
    (void)ImuMpu6050_RequestRead();
  }
}

void ImuMpu6050_GetState(imu_mpu6050_state_t *state)
{
  if (state != 0)
  {
    *state = imu_state;
  }
}

uint8_t ImuMpu6050_TestAddress(uint8_t addr7, uint8_t *who_am_i, uint32_t *hal_error)
{
  uint16_t addr = (uint16_t)addr7 << 1;
  uint8_t value = 0U;
  HAL_StatusTypeDef ready;
  HAL_StatusTypeDef read;
  uint8_t ok = 0U;

  if (ImuMpu6050_TryLock() == 0U)
  {
    if (who_am_i != 0)
    {
      *who_am_i = 0U;
    }
    if (hal_error != 0)
    {
      *hal_error = HAL_BUSY;
    }
    return 0U;
  }

  ready = HAL_I2C_IsDeviceReady(&hi2c1, addr, 1U, MPU6050_I2C_TIMEOUT_MS);
  if (ready != HAL_OK)
  {
    if (who_am_i != 0)
    {
      *who_am_i = 0U;
    }
    if (hal_error != 0)
    {
      *hal_error = HAL_I2C_GetError(&hi2c1);
    }
    ImuMpu6050_Unlock();
    return 0U;
  }

  read = HAL_I2C_Mem_Read(&hi2c1,
                          addr,
                          MPU6050_REG_WHO_AM_I,
                          I2C_MEMADD_SIZE_8BIT,
                          &value,
                          1U,
                          MPU6050_I2C_TIMEOUT_MS);
  if (who_am_i != 0)
  {
    *who_am_i = (read == HAL_OK) ? value : 0U;
  }
  if (hal_error != 0)
  {
    *hal_error = HAL_I2C_GetError(&hi2c1);
  }
  ok = (read == HAL_OK) ? 1U : 0U;
  ImuMpu6050_Unlock();
  return ok;
}

uint8_t ImuMpu6050_IsDeviceReady(uint8_t addr7)
{
  uint8_t ready;

  if (ImuMpu6050_TryLock() == 0U)
  {
    return 0U;
  }

  ready = (HAL_I2C_IsDeviceReady(&hi2c1,
                                 (uint16_t)addr7 << 1,
                                 1U,
                                 MPU6050_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
  ImuMpu6050_Unlock();
  return ready;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1 && imu_state.transfer_state == IMU_MPU6050_TRANSFER_READING)
  {
    imu_rx_done_ms = HAL_GetTick();
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_DONE;
    imu_rx_done = 1U;
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1 && imu_state.transfer_state == IMU_MPU6050_TRANSFER_READING)
  {
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_ERROR;
    imu_state.last_i2c_error = HAL_I2C_GetError(hi2c);
    imu_rx_error = 1U;
  }
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1 && imu_state.transfer_state == IMU_MPU6050_TRANSFER_READING)
  {
    imu_state.transfer_state = IMU_MPU6050_TRANSFER_ERROR;
    imu_state.last_i2c_error = HAL_I2C_GetError(hi2c);
    imu_rx_error = 1U;
  }
}
