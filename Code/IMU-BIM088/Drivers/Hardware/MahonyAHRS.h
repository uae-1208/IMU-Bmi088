//=====================================================================================================
// MahonyAHRS.h
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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

#define PI	   		3.1415926535f		
#define Q_angle     0.001f
#define Q_wbias     0.003f
#define R_measure   0.03f
#define dt          (1.0f / sampleFreq)

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame


typedef struct Euler_Angle{
    float roll;
    float pitch;
    float yaw;
}Euler_Angle;
//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void Data_Ope_Before_KF(float Acc_1, float Acc_2, float Gyro, float* Acc_angle, float* q_gyro);
void KF_Pitch(float Acc_angle, float q_gyro, float* pitch);
void KF_Roll(float Acc_angle, float q_gyro, float* roll);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
