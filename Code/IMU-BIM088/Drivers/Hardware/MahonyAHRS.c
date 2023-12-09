//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// Check: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions
//#define tau	 	0.4f				//��ΧΪ��㼸�뵽ʮ����
//#define beta	 	(2.416f / tau)		//https://zhuanlan.zhihu.com/p/582694093
//#define kp		(2.0f * beta)		// proportional gain
//#define ki		(beta * beta)		// integral gain
#define kp	   		0.5f				// proportional gain
#define ki			0.05f				// integral gain


#define sampleFreq	283.3f				// sample frequency in Hz
#define twoKpDef	(2.0f * kp)			// 2 * proportional gain
#define twoKiDef	(2.0f * ki)			// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float gx_offset=0.001848,gy_offset=0.001401,gz_offset=-0.002787;

//---------------------------------------------------------------------------------------------------
static float invSqrt(float x);

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	gx -= gx_offset;
	gy -= gy_offset;
	gz -= gz_offset;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		//���ٶȼƵõ���ʸ��a�Ĺ�һ�� 
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);  //invSqrt����Ľ���������ƽ��������
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		//�ɵ�λ��Ԫ���Ƶ���������ʸ��v
		//v����2�Լ�С��������֮��kp��ki��2���в���
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		//   (1-2(q1*q1+q2*q2))/2 = ((q0*q0+q3*q3)-(q1*q1+q2*q2))/2
		// = (2(q0*q0+q3*q3)-1)/2 
		halfvz = q0 * q0 - 0.5f + q3 * q3;              
	
		// Error is sum of cross product between estimated and measured direction of gravity
		//���ٶȵ�error = a x v  (������λʸ���Ĳ��)
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	//һ�����������������Ԫ��
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors   //0.5 * delta t
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	//qa��qb��qc�洢��һ�����ڵ�q0��q1��q2
	//��Ϊ�ڱ��ּ�����q0��q1��q2�����˸��£��������ڱ��ּ�����ֱ��ʹ��
	qa = q0;      
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	//�������ǽ��ٶȲ������ʹ��Ԫ��q��ģ������Ϊ1����Ҫ����ת���ɵ�λ��Ԫ��
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}





//KF
void Data_Ope_Before_KF(float Acc_1, float Acc_2, float Gyro, float* Acc_angle, float* q_gyro)
{
	*Acc_angle = atan((-Acc_1 / Acc_2)) / PI * 180;
	*q_gyro = Gyro / PI * 180;			//����ת�Ƕ�
}


void KF_Pitch(float Acc_angle, float q_gyro, float* pitch)
{
	static float q_bias;                     //��Ưֵ
	static float P[2][2]={{0,0},{0,0}};      //���Э�������
	static float PP[2][2];   			     //�������Э�������
	float K[2];                  		     //����������
	float den;                               //���������湫ʽ�еķ�ĸ
	float angle_err;                         //�۲�ֵ-H*�������(���ٶȼƽǶ�-�����ǽǶ�)
	
	/*Ԥ��*/
	*pitch += (q_gyro - q_bias) * dt;        //�������
	angle_err =  Acc_angle - *pitch;
	
	PP[0][0] = P[0][0] + dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);  //�������Э�������
	PP[0][1] = P[0][1] - dt * P[1][1];
	PP[1][0] = P[1][0] - dt * P[1][1];
   	PP[1][1] = P[1][1] + dt * Q_wbias;
	
	/*У��*/
	den = (P[0][0] + R_measure);             //����������
	K[0] = P[0][0] / den; 
	K[1] = P[0][1] / den;             
	
	*pitch += K[0] * angle_err;              //�����ں�
	q_bias += K[1] * angle_err;
	
	P[0][0] = (1 - K[0]) * PP[0][0];         //���Э�������
	P[0][1] = (1 - K[0]) * PP[0][1];
	P[1][0] = PP[1][0] -  K[1]* PP[0][0];
	P[1][1] = PP[1][1] -  K[1]* PP[0][1];
}

void KF_Roll(float Acc_angle, float q_gyro, float* roll)
{
	static float q_bias;                     //��Ưֵ
	static float P[2][2]={{0,0},{0,0}};      //���Э�������
	static float PP[2][2];   			     //�������Э�������
	float K[2];                  		     //����������
	float den;                               //���������湫ʽ�еķ�ĸ
	float angle_err;                         //�۲�ֵ-H*�������(���ٶȼƽǶ�-�����ǽǶ�)
	
	/*Ԥ��*/
	*roll += (q_gyro - q_bias) * dt;        //�������
	angle_err =  Acc_angle - *roll;
	
	PP[0][0] = P[0][0] + dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);  //�������Э�������
	PP[0][1] = P[0][1] - dt * P[1][1];
	PP[1][0] = P[1][0] - dt * P[1][1];
   	PP[1][1] = P[1][1] + dt * Q_wbias;
	
	/*У��*/
	den = (P[0][0] + R_measure);             //����������
	K[0] = P[0][0] / den; 
	K[1] = P[0][1] / den;             
	
	*roll += K[0] * angle_err;              //�����ں�
	q_bias += K[1] * angle_err;
	
	P[0][0] = (1 - K[0]) * PP[0][0];         //���Э�������
	P[0][1] = (1 - K[0]) * PP[0][1];
	P[1][0] = PP[1][0] -  K[1]* PP[0][0];
	P[1][1] = PP[1][1] -  K[1]* PP[0][1];
}



//====================================================================================================
// END OF CODE
//====================================================================================================
