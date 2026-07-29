#include "zf_common_headfile.h"
#include <stdio.h>

/*
 * 独立硬件链路测试：
 *   MSPM0G3507 天猛星 -> MPU6050 -> HC-05 -> 电脑
 *
 * MPU6050: SCL=PA1, SDA=PA0, I2C 地址 0x68
 * HC-05:   TX=PB2, RX=PB3, 9600 baud
 *
 * 电脑端每 100 ms 收到一行 CSV 原始数据：
 *   ax,ay,az,temp,gx,gy,gz
 */

#define MPU_ADDR            (0x68)
#define MPU_REG_SMPLRT_DIV  (0x19)
#define MPU_REG_CONFIG      (0x1A)
#define MPU_REG_GYRO_CONFIG (0x1B)
#define MPU_REG_ACCEL_CFG   (0x1C)
#define MPU_REG_DATA        (0x3B)
#define MPU_REG_PWR_MGMT_1  (0x6B)
#define MPU_REG_WHO_AM_I    (0x75)

static soft_iic_info_struct mpu_iic;

static int16 read_be16(const uint8 *data)
{
    return (int16)(((uint16)data[0] << 8) | data[1]);
}

static uint8 mpu6050_init(void)
{
    uint8 who;

    soft_iic_init(&mpu_iic, MPU_ADDR, 10, A1, A0);
    system_delay_ms(100);

    who = soft_iic_read_8bit_register(&mpu_iic, MPU_REG_WHO_AM_I);
    if((who != 0x68) && (who != 0x70))
    {
        return who;
    }

    soft_iic_write_8bit_register(&mpu_iic, MPU_REG_PWR_MGMT_1, 0x01);
    soft_iic_write_8bit_register(&mpu_iic, MPU_REG_SMPLRT_DIV, 0x07);
    soft_iic_write_8bit_register(&mpu_iic, MPU_REG_CONFIG, 0x03);
    soft_iic_write_8bit_register(&mpu_iic, MPU_REG_GYRO_CONFIG, 0x00);
    soft_iic_write_8bit_register(&mpu_iic, MPU_REG_ACCEL_CFG, 0x00);
    system_delay_ms(100);

    return who;
}

int main(void)
{
    uint8 who;
    uint8 raw[14];
    int16 ax, ay, az, temp, gx, gy, gz;
    char line[128];

    clock_init(SYSTEM_CLOCK_80M);
    uart_init(UART_3, 9600, UART3_TX_B2, UART3_RX_B3);
    system_delay_ms(300);

    uart_write_string(UART_3, "\r\nMSPM0G3507 MPU6050 HC05 TEST\r\n");
    who = mpu6050_init();

    sprintf(line, "WHO_AM_I=0x%02X\r\n", who);
    uart_write_string(UART_3, line);

    if((who != 0x68) && (who != 0x70))
    {
        uart_write_string(UART_3, "ERROR: MPU6050 NOT FOUND\r\n");
        while(1)
        {
            system_delay_ms(1000);
        }
    }

    uart_write_string(UART_3, "ax,ay,az,temp,gx,gy,gz\r\n");

    while(1)
    {
        soft_iic_read_8bit_registers(&mpu_iic, MPU_REG_DATA, raw, 14);

        ax   = read_be16(&raw[0]);
        ay   = read_be16(&raw[2]);
        az   = read_be16(&raw[4]);
        temp = read_be16(&raw[6]);
        gx   = read_be16(&raw[8]);
        gy   = read_be16(&raw[10]);
        gz   = read_be16(&raw[12]);

        sprintf(line, "%d,%d,%d,%d,%d,%d,%d\r\n",
                ax, ay, az, temp, gx, gy, gz);
        uart_write_string(UART_3, line);
        system_delay_ms(100);
    }
}
