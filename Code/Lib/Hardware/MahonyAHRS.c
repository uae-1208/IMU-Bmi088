//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
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
//#define tau	 	0.4f				//范围为零点几秒到十几秒
//#define beta	 	(2.416f / tau)		//https://zhuanlan.zhihu.com/p/582694093
//#define kp		(2.0f * beta)		// proportional gain
//#define ki		(beta * beta)		// integral gain
#define kp	   		0.5f				// proportional gain
#define ki			0.05f				// integral gain


//#define sampleFreq	283.3f				// sample frequency in Hz
#define sampleFreq	333.3f				// sample frequency in Hz
#define twoKpDef	(2.0f * kp)			// 2 * proportional gain
#define twoKiDef	(2.0f * ki)			// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
float gx_offset=0,gy_offset=0,gz_offset=0;

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
		//加速度计得到的矢量a的归一化 
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);  //invSqrt输出的结果是输入的平方根倒数
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		//由单位四元数推导出的重力矢量v
		//v被除2以减小运算量，之后被kp和ki乘2进行补偿
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		//   (1-2(q1*q1+q2*q2))/2 = ((q0*q0+q3*q3)-(q1*q1+q2*q2))/2
		// = (2(q0*q0+q3*q3)-1)/2 
		halfvz = q0 * q0 - 0.5f + q3 * q3;              
	
		// Error is sum of cross product between estimated and measured direction of gravity
		//角速度的error = a x v  (两个单位矢量的叉乘)
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
	//一节龙格库塔法更新四元数
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors   //0.5 * delta t
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	//qa、qb、qc存储上一个周期的q0、q1、q2
	//因为在本轮计算中q0、q1、q2发生了更新，不可以在本轮计算中直接使用
	qa = q0;      
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	//对陀螺仪角速度补偿后会使四元数q的模长不再为1，需要将其转换成单位四元数
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

//====================================================================================================
// END OF CODE
//====================================================================================================
