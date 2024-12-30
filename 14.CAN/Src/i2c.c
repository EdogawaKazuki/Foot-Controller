/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
/* Use Software instead of Hardware iic */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 **/
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL引脚时钟使能 */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA引脚时钟使能 */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 快速 */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* 开漏输出 */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

    iic_stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void iic_delay(void)
{
    HAL_Delay(2);    /* 2us的延时, 读写速度在250Khz以内 */
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
    iic_delay();
    IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
    iic_delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1); iic_delay();    /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    IIC_SCL(1); iic_delay();    /* SCL=1, 此时从机可以返回ACK */
    
    while (IIC_READ_SDA)    /* 等待应答 */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(void)
{
    IIC_SDA(0); iic_delay();    /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    IIC_SCL(1); iic_delay();    /* 产生一个时钟 */
    IIC_SCL(0); iic_delay();
    IIC_SDA(1); iic_delay();    /* 主机释放SDA线 */
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(void)
{
    IIC_SDA(1); iic_delay();    /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    IIC_SCL(1); iic_delay();    /* 产生一个时钟 */
    IIC_SCL(0); iic_delay();
    
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(uint8_t add)
{
    uint8_t t;
		IIC_SCL(0);
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((add & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        add <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(1);         /* 发送完成, 主机释放SDA线 */
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(1);
        iic_delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }
        
        IIC_SCL(0);
        iic_delay();
    }

    if (!ack)
    {
        iic_nack();     /* 发送nACK */
    }
    else
    {
        iic_ack();      /* 发送ACK */
    }

    return receive;
}
uint8_t iic_read_len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	iic_start(); 
	iic_send_byte((addr<<1)|0);//发送器件地址+写命令	
	if(iic_wait_ack())	//等待应答
	{
		iic_stop();		 
		return 1;		
	}
    iic_send_byte(reg);	//写寄存器地址
    iic_wait_ack();		//等待应答
    iic_start();
	iic_send_byte((addr<<1)|1);//发送器件地址+读命令	
    iic_wait_ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf = iic_read_byte(0);//读数据,发送nACK 
		else *buf = iic_read_byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
  iic_stop();	//产生一个停止条件 
	return 0;	
}

uint8_t iic_write_byte(uint8_t add, uint8_t data)
{
	iic_start();
	iic_send_byte((MPU_ADDR << 1) | 0); // Config the device address
	if(iic_wait_ack()) // wait_ack() = 1, return 1, enter the if and quit
	{
		iic_stop();
		return 1;
	}
	iic_send_byte(add); // Send one byte to the register address
	iic_wait_ack(); // Wait response from the reg
	iic_send_byte(data); // send the one byte_data to reg add
	if(iic_wait_ack())
	{
		iic_stop();
		return 1;
	}	
	iic_stop();
	return 0;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t iic_write_len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    iic_start(); 
	iic_send_byte((addr<<1)|0);//发送器件地址+写命令	
	if(iic_wait_ack())	//等待应答
	{
		iic_stop();		 
		return 1;		
	}
    iic_send_byte(reg);	//写寄存器地址
    iic_wait_ack();		//等待应答
	for(i=0;i<len;i++)
	{
		iic_send_byte(buf[i]);	//发送数据
		if(iic_wait_ack())		//等待ACK
		{
			iic_stop();	 
			return 1;		 
		}		
	}    
    iic_stop();	 
	return 0;	
} 

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return iic_write_byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return iic_write_byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return iic_write_byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=iic_write_byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=iic_read_len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=iic_read_len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

uint8_t MPU_Init(void)
{
	uint8_t res;
	iic_init();
	iic_write_byte(MPU_PWR_MGMT1_REG, 0X80);
	HAL_Delay(1000);
	iic_write_byte(MPU_PWR_MGMT1_REG, 0X00);
	MPU_Set_Gyro_Fsr(3);
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	iic_write_byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	iic_write_byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	iic_write_byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	iic_write_byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	res=iic_read_byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//器件ID正确
	{
		iic_write_byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		iic_write_byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}