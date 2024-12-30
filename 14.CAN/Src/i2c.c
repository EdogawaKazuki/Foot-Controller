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
 * @brief       ��ʼ��IIC
 * @param       ��
 * @retval      ��
 **/
void iic_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_SCL_GPIO_CLK_ENABLE();  /* SCL����ʱ��ʹ�� */
    IIC_SDA_GPIO_CLK_ENABLE();  /* SDA����ʱ��ʹ�� */

    gpio_init_struct.Pin = IIC_SCL_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;        /* ������� */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* ���� */
    HAL_GPIO_Init(IIC_SCL_GPIO_PORT, &gpio_init_struct);/* SCL */

    gpio_init_struct.Pin = IIC_SDA_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;        /* ��©��� */
    HAL_GPIO_Init(IIC_SDA_GPIO_PORT, &gpio_init_struct);/* SDA */
    /* SDA����ģʽ����,��©���,����, �����Ͳ���������IO������, ��©�����ʱ��(=1), Ҳ���Զ�ȡ�ⲿ�źŵĸߵ͵�ƽ */

    iic_stop();     /* ֹͣ�����������豸 */
}

/**
 * @brief       IIC��ʱ����,���ڿ���IIC��д�ٶ�
 * @param       ��
 * @retval      ��
 */
static void iic_delay(void)
{
    HAL_Delay(2);    /* 2us����ʱ, ��д�ٶ���250Khz���� */
}

/**
 * @brief       ����IIC��ʼ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);     /* START�ź�: ��SCLΪ��ʱ, SDA�Ӹ߱�ɵ�, ��ʾ��ʼ�ź� */
    iic_delay();
    IIC_SCL(0);     /* ǯסI2C���ߣ�׼�����ͻ�������� */
    iic_delay();
}

/**
 * @brief       ����IICֹͣ�ź�
 * @param       ��
 * @retval      ��
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP�ź�: ��SCLΪ��ʱ, SDA�ӵͱ�ɸ�, ��ʾֹͣ�ź� */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* ����I2C���߽����ź� */
    iic_delay();
}

/**
 * @brief       �ȴ�Ӧ���źŵ���
 * @param       ��
 * @retval      1������Ӧ��ʧ��
 *              0������Ӧ��ɹ�
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1); iic_delay();    /* �����ͷ�SDA��(��ʱ�ⲿ������������SDA��) */
    IIC_SCL(1); iic_delay();    /* SCL=1, ��ʱ�ӻ����Է���ACK */
    
    while (IIC_READ_SDA)    /* �ȴ�Ӧ�� */
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, ����ACK��� */
    iic_delay();
    return rack;
}

/**
 * @brief       ����ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_ack(void)
{
    IIC_SDA(0); iic_delay();    /* SCL 0 -> 1 ʱ SDA = 0,��ʾӦ�� */
    IIC_SCL(1); iic_delay();    /* ����һ��ʱ�� */
    IIC_SCL(0); iic_delay();
    IIC_SDA(1); iic_delay();    /* �����ͷ�SDA�� */
}

/**
 * @brief       ������ACKӦ��
 * @param       ��
 * @retval      ��
 */
void iic_nack(void)
{
    IIC_SDA(1); iic_delay();    /* SCL 0 -> 1  ʱ SDA = 1,��ʾ��Ӧ�� */
    IIC_SCL(1); iic_delay();    /* ����һ��ʱ�� */
    IIC_SCL(0); iic_delay();
    
}

/**
 * @brief       IIC����һ���ֽ�
 * @param       data: Ҫ���͵�����
 * @retval      ��
 */
void iic_send_byte(uint8_t add)
{
    uint8_t t;
		IIC_SCL(0);
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((add & 0x80) >> 7);    /* ��λ�ȷ��� */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        add <<= 1;     /* ����1λ,������һ�η��� */
    }
    IIC_SDA(1);         /* �������, �����ͷ�SDA�� */
}

/**
 * @brief       IIC��ȡһ���ֽ�
 * @param       ack:  ack=1ʱ������ack; ack=0ʱ������nack
 * @retval      ���յ�������
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    for (i = 0; i < 8; i++ )    /* ����1���ֽ����� */
    {
        receive <<= 1;  /* ��λ�����,�������յ�������λҪ���� */
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
        iic_nack();     /* ����nACK */
    }
    else
    {
        iic_ack();      /* ����ACK */
    }

    return receive;
}
uint8_t iic_read_len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	iic_start(); 
	iic_send_byte((addr<<1)|0);//����������ַ+д����	
	if(iic_wait_ack())	//�ȴ�Ӧ��
	{
		iic_stop();		 
		return 1;		
	}
    iic_send_byte(reg);	//д�Ĵ�����ַ
    iic_wait_ack();		//�ȴ�Ӧ��
    iic_start();
	iic_send_byte((addr<<1)|1);//����������ַ+������	
    iic_wait_ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf = iic_read_byte(0);//������,����nACK 
		else *buf = iic_read_byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
  iic_stop();	//����һ��ֹͣ���� 
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
//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
uint8_t iic_write_len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    iic_start(); 
	iic_send_byte((addr<<1)|0);//����������ַ+д����	
	if(iic_wait_ack())	//�ȴ�Ӧ��
	{
		iic_stop();		 
		return 1;		
	}
    iic_send_byte(reg);	//д�Ĵ�����ַ
    iic_wait_ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		iic_send_byte(buf[i]);	//��������
		if(iic_wait_ack())		//�ȴ�ACK
		{
			iic_stop();	 
			return 1;		 
		}		
	}    
    iic_stop();	 
	return 0;	
} 

//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return iic_write_byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return iic_write_byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return iic_write_byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=iic_write_byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
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
//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
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
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	iic_write_byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	iic_write_byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	iic_write_byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	iic_write_byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=iic_read_byte(MPU_DEVICE_ID_REG); 
	if(res==MPU_ADDR)//����ID��ȷ
	{
		iic_write_byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		iic_write_byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}