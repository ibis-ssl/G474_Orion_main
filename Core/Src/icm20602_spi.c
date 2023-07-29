
#include "icm20602_spi.h"





float weight[2]   = {0.5f, 0.5f};

float acc[3];
float acc_comp[3];
float gyro_comp[3];

float imu_temperature;

float gyro[3];
float gyro_off[3];
float acc_off[3];

// Acc Full Scale Range  +-2G 4G 8G 16G 
enum Ascale
{
    AFS_2G=0,  
    AFS_4G,
    AFS_8G,
    AFS_16G
};

// Gyro Full Scale Range +-250 500 1000 2000 Degrees per second
enum Gscale
{
    GFS_250DPS=0,   
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
};

// Scale resolutions per LSB for the sensors
int Ascale = AFS_2G;
int Gscale = GFS_1000DPS;

float aRes,gRes;

void ICM20602_writeByte(uint8_t reg, uint8_t data)
{
	uint8_t send_data[1];
	uint8_t RxBuffer[1];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

	send_data[0]=reg & 0x7F;
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)send_data,(uint8_t*)RxBuffer,1,2000);

	send_data[0]=data;
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)send_data,(uint8_t*)RxBuffer,1,2000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

uint8_t ICM20602_readByte(uint8_t reg)
{
    uint8_t val;
    uint8_t send_data[1];
    uint8_t RxBuffer[1];
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

    send_data[0]= reg | 0x80;
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)send_data,(uint8_t*)RxBuffer,1,2000);

    send_data[0]=0x00;
    HAL_SPI_TransmitReceive(&hspi1,(uint8_t*)send_data,(uint8_t*)RxBuffer,1,2000);
    val = RxBuffer[0];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
    
    return (val);
}

// Communication test: WHO_AM_I register reading 
uint16_t ICM20602_getWhoAmI()
{
    return ICM20602_readByte(ICM20602_WHO_AM_I);   // Should return 0x68
}

void ICM20602_init()
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1);
	ICM20602_writeByte(ICM20602_PWR_MGMT_1, 0x00);    // CLK_SEL=0: internal 8MHz, TEMP_DIS=0, SLEEP=0
	ICM20602_writeByte(ICM20602_SMPLRT_DIV, 0x07);  // Gyro output sample rate = Gyro Output Rate/(1+SMPLRT_DIV)
	ICM20602_writeByte(ICM20602_CONFIG, 0x01); //176Hz     // set TEMP_OUT_L, DLPF=3 (Fs=1KHz):0x03

	ICM20602_setAccRange(Ascale);
	ICM20602_setGyroRange(Gscale);
}

int16_t ICM20602_getAccXvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_ACCEL_XOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_ACCEL_XOUT_H); // read Accelerometer X_High value
    return((HiByte<<8) | LoByte);
}

int16_t ICM20602_getAccYvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_ACCEL_YOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_ACCEL_YOUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t ICM20602_getAccZvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_ACCEL_ZOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_ACCEL_ZOUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t ICM20602_getGyrXvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_GYRO_XOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_GYRO_XOUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t ICM20602_getGyrYvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_GYRO_YOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_GYRO_YOUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t ICM20602_getGyrZvalue()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_GYRO_ZOUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_GYRO_ZOUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}

int16_t ICM20602_getIMUTemp()
{
    uint8_t LoByte, HiByte;
    LoByte = ICM20602_readByte(ICM20602_TEMP_OUT_L); // read Accelerometer X_Low  value
    HiByte = ICM20602_readByte(ICM20602_TEMP_OUT_H); // read Accelerometer X_High value
    return ((HiByte<<8) | LoByte);
}


// Calculates Acc resolution
float ICM20602_setAccRange(int Ascale)
{

    switch(Ascale)
    {
        case AFS_2G:
            aRes = 2.0/32768.0;
            break;
        case AFS_4G:
            aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            aRes = 8.0/32768.0;
            break;
        case AFS_16G:
            aRes = 16.0/32768.0;
            break;         
    }

    ICM20602_writeByte(ICM20602_ACCEL_CONFIG, Ascale<<3);// bit[4:3] 0=+-2g,1=+-4g,2=+-8g,3=+-16g, ACC_HPF=On (5Hz)
    
    return aRes;
}

// Calculates Gyro resolution
float ICM20602_setGyroRange(int Gscale)
{
    switch (Gscale) {
        case GFS_250DPS:
            gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }

    ICM20602_writeByte(ICM20602_GYRO_CONFIG, Gscale<<3); // bit[4:3] 0=+-250d/s,1=+-500d/s,2=+-1000d/s,3=+-2000d/s
    
    return gRes;
}

int ICM20602_getAccRange(void)
{
    int Ascale;

    Ascale = ICM20602_readByte(ICM20602_ACCEL_CONFIG);
    Ascale = (Ascale & 0x18) >> 3;

    return Ascale;
}

int ICM20602_getGyroRange(void)
{
    int Gscale;

    Gscale = ICM20602_readByte(ICM20602_GYRO_CONFIG);
    Gscale = (Gscale & 0x18) >> 3;

    return Gscale;
}

void ICM20602_read_IMU_data(float imu_dt_sec)
{
    static float gyro_prv[3] = {0.0f};

    acc[0] = ICM20602_getAccXvalue() * IMU_ONE_G * aRes;
    acc[1] = ICM20602_getAccYvalue() * IMU_ONE_G * aRes;
    acc[2] = ICM20602_getAccZvalue() * IMU_ONE_G * aRes;
    gyro[0] = ICM20602_getGyrXvalue() * gRes;
    gyro[1] = ICM20602_getGyrYvalue() * gRes;
    gyro[2] = ICM20602_getGyrZvalue() * gRes;

    ICM20602_medianFilter();

    imu_temperature = (ICM20602_getIMUTemp() / 326.8f) + 25.0f;
    ICM20602_IMU_compensate();

    pitch_angle = pitch_angle + ICM20602_integral(gyro_comp[0], gyro_prv[0], imu_dt_sec) * 1;  //とりあえず変えておく
    roll_angle = roll_angle + ICM20602_integral(gyro_comp[1], gyro_prv[1], imu_dt_sec) * 1;
    yaw_angle = yaw_angle + ICM20602_integral(gyro_comp[2], gyro_prv[2], imu_dt_sec) * 1;

    pitch_angle = ICM20602_normAngle(pitch_angle);
    roll_angle  = ICM20602_normAngle(roll_angle);
    yaw_angle   = ICM20602_normAngle(yaw_angle);

    gyro_prv[0] = gyro_comp[0];
    gyro_prv[1] = gyro_comp[1];
    gyro_prv[2] = gyro_comp[2];
}

float ICM20602_integral(float val, float val_prv, float dt)
{
    return (val + val_prv) * dt / 2.0f;   // trapezoidal formula
}

void ICM20602_clearAngle(void)
{
    pitch_angle = 0.0f;
    roll_angle  = 0.0f;
    yaw_angle   = 0.0f;
}

void ICM20602_setAngle(float pitch, float roll, float yaw)
{
    pitch_angle = pitch;
    roll_angle  = roll;
    yaw_angle   = yaw;
}

float ICM20602_normAngle(float deg)
{
    while (deg < -180.0f) deg += 360.0f;
    while (deg >= 180.0f) deg -= 360.0f;

    return deg;
}

float ICM20602_complementaryWeight(float first, float second)
{
    float weights_ratio = 1.0f / (first + second);

    weight[0] = first  * weights_ratio;
    weight[1] = second * weights_ratio;

    return yaw_angle;
}

float ICM20602_complementaryFilter(float val)
{
    float yaw_temp = yaw_angle;

    if     (yaw_temp - val >= 180.0f)  val      += 360.0f;
    else if(yaw_temp - val < -180.0f)  yaw_temp += 360.0f;

    yaw_temp = val * weight[0] + yaw_temp * weight[1];

    yaw_angle = ICM20602_normAngle(yaw_temp);

    return yaw_angle;
}

// filter length : 3-only
void ICM20602_medianFilter(void)
{
    static float acc_mdat[3][3] = {{0.0f}};
    static float gyro_mdat[3][3] = {{0.0f}};
    float gyro_tmp[3] = {0.0f};
    float acc_tmp[3] = {0.0f};

    float tmp;
    int8_t i, j, a, b;

    for (i = 0; i < 3; i ++) {

    	gyro_mdat[i][2] = gyro_mdat[i][1];
    	gyro_mdat[i][1] = gyro_mdat[i][0];
    	gyro_mdat[i][0] = gyro[i];

    	acc_mdat[i][2]  = acc_mdat[i][1];
    	acc_mdat[i][1]  = acc_mdat[i][0];
    	acc_mdat[i][0]  = acc[i];

    	for (j = 0; j < 3; j ++){
    		gyro_tmp[j] = gyro_mdat[i][j];
    		acc_tmp[j]  = acc_mdat[i][j];
    	}

    	a = 0;
    	b = 2;

    	for (j = 2; j >= 0; j--) {
    		if (gyro_tmp[a] > gyro_tmp[b]) {
    			tmp         = gyro_tmp[a];
    			gyro_tmp[a] = gyro_tmp[b];
    			gyro_tmp[b] = tmp;
    		}
    		if (acc_tmp[a]  > acc_tmp[b]) {
				tmp         = acc_tmp[a];
				acc_tmp[a]  = acc_tmp[b];
				acc_tmp[b]  = tmp;
			}
    		a = j-1;
    		b = j;
    	}
    	gyro[i] = gyro_tmp[1];
    	acc[i]  = acc_tmp[1];
    }
}

void ICM20602_IMU_calibration(void)
{
	int i,j;
    double cal_len = 5000.0f;
    double acc_sum[3]={0};
    double gyro_sum[3]={0};

    printf("put the IMU still!\n");

    for(i = 0; i < (int)cal_len; i ++){
    		acc[0] = ICM20602_getAccXvalue() * IMU_ONE_G * aRes;
    		acc[1] = ICM20602_getAccYvalue() * IMU_ONE_G * aRes;
    		acc[2] = ICM20602_getAccZvalue() * IMU_ONE_G * aRes;
    		gyro[0] = ICM20602_getGyrXvalue() * gRes;
    		gyro[1] = ICM20602_getGyrYvalue() * gRes;
    		gyro[2] = ICM20602_getGyrZvalue() * gRes;
    		delayUs(300);
    }
    for(i = 0; i < (int)cal_len; i ++){
    	for(j = 0; j < 3; j++){
    		acc[0] = ICM20602_getAccXvalue() * IMU_ONE_G * aRes;
    		acc[1] = ICM20602_getAccYvalue() * IMU_ONE_G * aRes;
    		acc[2] = ICM20602_getAccZvalue() * IMU_ONE_G * aRes;
    		gyro[0] = ICM20602_getGyrXvalue() * gRes;
    		gyro[1] = ICM20602_getGyrYvalue() * gRes;
    		gyro[2] = ICM20602_getGyrZvalue() * gRes;

    		acc_sum[j]  += (double)acc[j];
    		gyro_sum[j] += (double)gyro[j];
    		delayUs(300);
    	}
    }

    for(j = 0; j < 3; j++){
    	acc_off[j]  = acc_sum[j]  / cal_len;
    	gyro_off[j] = gyro_sum[j] / cal_len;
    }
}

void ICM20602_IMU_calibration2(void)
{
	int i,j;
    double cal_len = 0.0f;
    double acc_sum[3]={0};
    double gyro_sum[3]={0};

    double acc_ave[3][10] = {{0.0}};
    double gyro_ave[3][10] = {{0.0}};

    printf("put the IMU still!\n");
    HAL_Delay(1000);

    while ((fabs(acc_ave[0][9] - acc_ave[0][0]) > SHRINK_ERROR)
		|| (fabs(acc_ave[1][9] - acc_ave[1][0]) > SHRINK_ERROR)
		|| (fabs(acc_ave[2][9] - acc_ave[2][0]) > SHRINK_ERROR)
		|| (fabs(gyro_ave[0][9] - gyro_ave[0][0]) > SHRINK_ERROR)
		|| (fabs(gyro_ave[1][9] - gyro_ave[1][0]) > SHRINK_ERROR)
		|| (fabs(gyro_ave[2][9] - gyro_ave[2][0]) > SHRINK_ERROR)
		|| cal_len < 500.0
		){

    	printf("cal_len=%f\r\n",cal_len);
    	for(j = 0; j < 3; j++){
    		acc[0] = ICM20602_getAccXvalue() * IMU_ONE_G * aRes;
    		acc[1] = ICM20602_getAccYvalue() * IMU_ONE_G * aRes;
    		acc[2] = ICM20602_getAccZvalue() * IMU_ONE_G * aRes;
    		gyro[0] = ICM20602_getGyrXvalue() * gRes;
    		gyro[1] = ICM20602_getGyrYvalue() * gRes;
    		gyro[2] = ICM20602_getGyrZvalue() * gRes;

    		acc_sum[j]  += acc[j];
    		gyro_sum[j] += gyro[j];

    		acc_ave[j][0]  = acc_sum[j]  / cal_len;
    		gyro_ave[j][0] = gyro_sum[j] / cal_len;

    		for(i = 9; i > 0; i --){
    			acc_ave[j][i] = acc_ave[j][i-1];
    			gyro_ave[j][i] = gyro_ave[j][i-1];
    		}
    	}
    	cal_len ++;

    	if(cal_len>5000){break;}
    }

    if(cal_len==500){NVIC_SystemReset();}

    for(j = 0; j < 3; j++){
    	acc_off[j]  = acc_sum[j]  / cal_len;
    	gyro_off[j] = gyro_sum[j] / cal_len;
    }
}

void ICM20602_IMU_compensate(void)
{
    int k;
    for(k=0;k<3;k++){
        acc_comp[k] = acc[k] - acc_off[k];
        gyro_comp[k] = gyro[k] - gyro_off[k];
    }

}



