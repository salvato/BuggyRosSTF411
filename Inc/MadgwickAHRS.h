//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick {
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(void);
    void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void update(float* g, float* a, float* m);

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    float getRoll();
    float getPitch();
    float getYaw();

    float getRollRadians();
    float getPitchRadians();
    float getYawRadians();

    void getRotation(float* r0, float* r1, float* r2, float* r3);
    void getRotation(float* r);

    void getGravity(float *vx, float *vy, float *vz);
    void getGravity(float *v);
};
#endif

