/*
// To configure QtCreator in order to Debug and Run programs
// on STM32F boards see:
//
// https://github.com/nlhans/qt-baremetal
// https://electronics.stackexchange.com/questions/212018/debugging-an-arm-stm32-microcontroller-using-qt-creator
//
//
// Motor Gear 9:1
//
// Tire's Diameter 0.069 [m]
// Encoder Counts per Tire Turn = 12.0*9.0*4.0 = 432 ==>
// Encoder Counts per [m] = 1993
//
// Tire's Distance 0.2 [m]

//=========================================
// On Board Peripherals
//=========================================
// PA5  (CN10 11)   ------> Led On Board
// PC13 (CN7  23)   ------> Button On Board


//===================================
// USART2 GPIO Configuration
//===================================
// GND (CN10 20)    ------> GND
// PA2 (CN10 35)    ------> USART2_TX
// PA3 (CN10 37)    ------> USART2_RX


//===================================
// I2C2 GPIO Configuration
//===================================
// VIN  (CN7  16)    ------> +3.3V
// GND  (CN7  20)    ------> GND
// PB10 (CN10 25)    ------> I2C2_SCL
// PB3  (CN10 31)    ------> I2C2_SDA


//================================================
// TIM1 (32 Bit) GPIO Configuration (Left Encoder)
//================================================
// PA8 (CN10 21)    ------> TIM1_CH1
// PA9 (CN10 23)    ------> TIM1_CH2


//=================================================
// TIM4 (32 Bit) GPIO Configuration (Right Encoder)
//=================================================
// PB6 (CN10 17)    ------> TIM4_CH1
// PB7 (CN7  21)    ------> TIM4_CH2


//=============================================
// TIM2 GPIO Configuration (Periodic Interrupt)
//=============================================
// Channel2         ------> Motors Sampling
// Channel3         ------> Sonar Sampling
// Channel4         ------> AHRS Sampling


//==================================
// TIM3 GPIO Configuration (PWM)
//==================================
// PA6 (CN10 13)    ------> TIM3_CH1
// PA7 (CN10 15)    ------> TIM3_CH2


//==========================================
// TIM5 GPIO Configuration (Sonar)
//==========================================
// PA0 (CN7 - 28)  ------> TIM3_CH1 (Pulse)
// PA1 (CN7 - 30)  ------> TIM3_CH2 (Echo)


//====================================
// Left Motor Direction Pins
// PC8  (CN10  2)    ------> LM298 IN1
// PC9  (CN10  1)    ------> LM298 IN2
//====================================


//====================================
// Right Motor Direction Pins
// PC10 (CN7  1)    ------> LM298 IN3
// PC11 (CN7  2)    ------> LM298 IN4
//====================================

//====================================
// Used Peripherals:
// Timers:
//     TIM1 ---> Encoder (Left)
//     TIM2 ---> Periodic Interrupt
//     TIM3 ---> PWM (Motors)
//     TIM4 ---> Encoder (Right)
//     TIM9 ---> Ultrasound Sensor
//====================================

//=============================================================================
// ROS information:
// http://wiki.ros.org/navigation/Tutorials/RobotSetup
//=============================================================================
*/

#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "uart.h"
#include "utility.h"
#include "encoder.h"
#include "dcmotor.h"
#include "PID_v1.h"
#include "controlledmotor.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"
#include "string.h" // for memset()
#ifdef  USE_FULL_ASSERT
#include "stdio.h"
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Range.h>


#define MOTOR_UPDATE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_2
#define SONAR_UPDATE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_3
#define AHRS_UPDATE_CHANNEL  HAL_TIM_ACTIVE_CHANNEL_4


///==============================
/// Private function prototypes
///==============================
static void Setup();
static void Loop();
static void Init_Hardware();
static bool AHRS_Init();
static void Init_ROS();


///================
/// ROS callbacks
///================
static void targetSpeed_cb(const geometry_msgs::Twist& speed);
static void left_PID_cb(const geometry_msgs::Vector3& msg);
static void right_PID_cb(const geometry_msgs::Vector3& msg);


///===================
/// Hardware Handles
///===================
TIM_HandleTypeDef  hSamplingTimer;     // Periodic Sampling Timer
TIM_HandleTypeDef  hLeftEncoderTimer;  // Left Motor Encoder Timer
TIM_HandleTypeDef  hRightEncoderTimer; // Right Motor Encoder Timer
TIM_HandleTypeDef  hPwmTimer;          // Dc Motors PWM (Speed control)
TIM_HandleTypeDef  hSonarEchoTimer;    // To Measure the Radar Echo Pulse Duration
TIM_HandleTypeDef  hSonarPulseTimer;   // To Generate the Radar Trigger Pulse

I2C_HandleTypeDef  hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx;


///==================
/// Buggy Mechanics
///==================
static const double WHEEL_DIAMETER               = 0.069;  // [m]
static const double WHEEL_RADIUS                 = 0.5 * WHEEL_DIAMETER;
static const int    ENCODER_COUNTS_PER_TIRE_TURN = 12*9*4; // = 432 ==>
static const double TRACK_LENGTH                 = 0.2;    // Tire's Distance [m]


///===================
/// Shared variables
///===================
unsigned int baudRate = 57600;

double periodicClockFrequency = 10.0e6;  // 10MHz
double pwmClockFrequency      = 20.0e3;  // 10KHz
double sonarClockFrequency    = 10.0e6;  // 10MHz (100ns period)
double sonarPulseDelay        = 10.0e-6; // in seconds
double sonarPulseWidth        = 10.0e-6; // in seconds
double soundSpeed             = 340.0;   // in m/s

double encoderCountsPerMeter  = ENCODER_COUNTS_PER_TIRE_TURN/(M_PI*WHEEL_DIAMETER);


///====================
/// Private variables
///====================
Encoder*         pLeftEncoder          = nullptr;
Encoder*         pRightEncoder         = nullptr;
DcMotor*         pLeftMotor            = nullptr;
DcMotor*         pRightMotor           = nullptr;
ControlledMotor* pLeftControlledMotor  = nullptr;
ControlledMotor* pRightControlledMotor = nullptr;

// Inital values for PIDs (externally estimated)
static const double LeftP  = 0.049;
static const double LeftI  = 0.002;
static const double LeftD  = 0.001;

static const double RightP = 0.101;
static const double RightI = 0.008;
static const double RightD = 0.003;

ADXL345  Acc;      // 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     // 400KHz I2C Capable
HMC5883L Magn;     // 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; // ~13us per Madgwick.update() with NUCLEO-F411RE

static float q0, q1, q2, q3;
static float AccelValues[3];
static float GyroValues[3];
static float MagValues[3];

static double pastPosition[2] = {0.0};
static double pastOrientation = 0.0;

///===================
/// Update Intervals
///===================
uint32_t AHRSSamplingFrequency   = 400; // [Hz]
uint32_t motorSamplingFrequency  = 50;  // [Hz]
uint32_t sonarSamplingFrequency  = 10;  // [Hz] (Max 40Hz)

uint32_t AHRSSamplingPulses  = uint32_t(periodicClockFrequency/AHRSSamplingFrequency +0.5); // [Hz]
uint32_t motorSamplingPulses = uint32_t(periodicClockFrequency/motorSamplingFrequency+0.5); // [Hz]
uint32_t sonarSamplingPulses = uint32_t(periodicClockFrequency/sonarSamplingFrequency+0.5); // [Hz]

bool bAHRSpresent    = false;

bool isTimeToUpdateAHRS   = false;
bool isTimeToUpdateMotors = false;
bool isTimeToUpdateSonar  = false;

/// Captured Values for Echo Width Calculation
volatile uint32_t uwIC2Value1    = 0;
volatile uint32_t uwIC2Value2    = 0;
volatile uint32_t uwDiffCapture  = 0;
volatile uint16_t uhCaptureIndex = 0;

double leftTargetSpeed  = 0.0;
double rightTargetSpeed = 0.0;


///=============
/// ROS stuff
///=============
// in ros.h
//template<class Hardware,
//         int MAX_SUBSCRIBERS = 25,
//         int MAX_PUBLISHERS  = 25,
//         int INPUT_SIZE      = 1004,
//         int OUTPUT_SIZE     = 1024>
//class NodeHandle_
//typedef NodeHandle_<STM32Hardware> NodeHandle;
ros::NodeHandle nh;

ros::Time last_cmd_vel_time;
ros::Time prev_update_time;

nav_msgs::Odometry               odom;
geometry_msgs::Quaternion        actual_rotation;
geometry_msgs::Vector3Stamped    actual_speed;
sensor_msgs::Range               obstacleDistance;

ros::Publisher rotation_pub("buggyRotation", &actual_rotation);
ros::Publisher actual_speed_pub("buggySpeed", &actual_speed);
ros::Publisher obstacleDistance_pub("buggyDistance", &obstacleDistance);

ros::Subscriber<geometry_msgs::Twist>   targetSpeed_sub("cmd_vel", &targetSpeed_cb);
ros::Subscriber<geometry_msgs::Vector3> left_PID_sub("leftPID", &left_PID_cb);
ros::Subscriber<geometry_msgs::Vector3> right_PID_sub("rightPID", &right_PID_cb);


///=======================================================================
///                                Main
///=======================================================================
int
main(void) {
    Setup();
    while(true) {
        Loop();
    }
}


///=======================================================================
///                                Init
///=======================================================================
static void
Setup() {
    Init_Hardware();
    Init_ROS();
    // Enable the Periodic Samplig Timer Interrupt
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    // Start the Periodic Sampling of: AHRS, Motors and Sonar
    HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_4);
    HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&hSamplingTimer, TIM_CHANNEL_3);
    // Enable and set Button EXTI Interrupt
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    prev_update_time = nh.now();
    nh.loginfo("Buggy Ready...");
}


///=======================================================================
///    Main Loop
///=======================================================================
static void
Loop() {
    if(isTimeToUpdateAHRS) {
        isTimeToUpdateAHRS = false;
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        Madgwick.getRotation(&q0, &q1, &q2, &q3);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        actual_rotation.w = q0;
        actual_rotation.x = q1;
        actual_rotation.y = q2;
        actual_rotation.z = q3;
    }

    if(isTimeToUpdateMotors) {
        isTimeToUpdateMotors = false;
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        actual_speed.header.stamp = nh.now();
/// TODO:
/// Da calcolare velocità lineare ed Angolare !!!
        actual_speed.vector.x = pLeftControlledMotor->getCurrentSpeed();
        actual_speed.vector.y = pRightControlledMotor->getCurrentSpeed();
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }

    if(isTimeToUpdateSonar) {
        isTimeToUpdateSonar = false;
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
        double distance = 0.5 * soundSpeed*(double(uwDiffCapture)/sonarClockFrequency); // [m]
        obstacleDistance.header.stamp = nh.now();
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        obstacleDistance.radiation_type = obstacleDistance.ULTRASOUND;
        obstacleDistance.range = distance;

        static double pastPosition[2];
        static double pastOrientation;

        odom.pose.pose.position.x = ;//odom_pose_[0];
        odom.pose.pose.position.y = ;//odom_pose_[1];
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = actual_rotation;

        // We should update the twist of the odometry
        odom.twist.twist.linear.x  = ;//odom_vel_[0];
        odom.twist.twist.angular.z = ;//odom_vel_[2];

        rotation_pub.publish(&actual_rotation);
        actual_speed_pub.publish(&actual_speed);
        obstacleDistance_pub.publish(&obstacleDistance);
        //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    nh.spinOnce();
}
///=======================================================================
///    End Loop
///=======================================================================

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
#define LEFT  0
#define RIGHT 1
double odom_pose_[3] = {0.0};
double odom_vel_[3] = {0.0};
bool
updateOdometry(ros::Duration diff_time) {
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = leftTargetSpeed;
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r

  v[RIGHT] = rightTargetSpeed;
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

//  last_velocity_[LEFT]  = w[LEFT];
//  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = pLeftControlledMotor->spaceTraveled();  // Spazio percorso dalla ruota sinistra
  wheel_r = pRightControlledMotor->spaceTraveled(); // Spazio percorso dalla ruota destra

//  last_position_[LEFT]  += wheel_l;
//  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0; // Spazio percorso dal centro del Robot
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / TRACK_LENGTH; // Rotazione dell'asse del Robot

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = nh.createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom.twist.twist.linear.x  = odom_vel_[0];
  odom.twist.twist.angular.z = odom_vel_[2];

  return true;
}


void
Init_Hardware() {
    HAL_Init();           // Initialize the HAL Library
    SystemClock_Config(); // Initialize System Clock
    GPIO_Init();          // Initialize On Board Peripherals
    SerialPortInit();     // Initialize the Serial Communication Port (/dev/ttyACM0)

    // Initialize Encoders
    LeftEncoderTimerInit();
    RightEncoderTimerInit();
    pLeftEncoder  = new Encoder(&hLeftEncoderTimer);
    pRightEncoder = new Encoder(&hRightEncoderTimer);

    // Initialize Dc Motors
    PwmTimerInit();
    // =============> DcMotor(forwardPort, forwardPin, reversePort,  reversePin,
    //                        pwmPort,  pwmPin, pwmTimer, timerChannel)
    pLeftMotor  = new DcMotor(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9,
                              GPIOA, GPIO_PIN_6, &hPwmTimer, TIM_CHANNEL_1);
    pRightMotor = new DcMotor(GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11,
                              GPIOA, GPIO_PIN_7, &hPwmTimer, TIM_CHANNEL_2);

    // Initialize Motor Controllers
    pLeftControlledMotor  = new ControlledMotor(pLeftMotor,  pLeftEncoder,  motorSamplingFrequency);
    pRightControlledMotor = new ControlledMotor(pRightMotor, pRightEncoder, motorSamplingFrequency);
    pLeftControlledMotor->setPID(LeftP, LeftI, LeftD);
    pRightControlledMotor->setPID(RightP, RightI, RightD);

    // Initialize 10DOF Sensor
    I2C2_Init();
    bAHRSpresent = AHRS_Init();

    // Initialize Sonar
    SonarEchoTimerInit();
    SonarPulseTimerInit();

    // Initialize Periodic Samplig Timer
    SamplingTimerInit(AHRSSamplingPulses, motorSamplingPulses, sonarSamplingPulses);
}


bool
AHRS_Init() {
    if(!Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c2))
        return false;

    if(!Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c2))
        return false;
    HAL_Delay(100);
     // Calibrate the ITG3200 (Assuming a STATIC SENSOR !)
    Gyro.zeroCalibrate(1200);

    if(!Magn.init(HMC5883L_Address, &hi2c2))
        return false;
    HAL_Delay(100);
    if(Magn.SetScale(1300) != 0)
        return false;
    HAL_Delay(100);
    if(Magn.SetMeasurementMode(Measurement_Continuous) != 0)
        return false;

    Madgwick.begin(float(AHRSSamplingFrequency));
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(AccelValues);
    while(!Gyro.isRawDataReadyOn()) {}
    Gyro.readGyro(GyroValues);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(MagValues);

    // Since we start with an arbitrary orientation we have to converge to
    // the initial estimate of the attitude (assuming a static sensor !)
    for(int i=0; i<20000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
        Madgwick.update(GyroValues, AccelValues, MagValues);
    }
    pastOrientation = Madgwick.getYaw();
    return true;
}


void
Init_ROS() {
    nh.initNode();

    if(!nh.advertise(rotation_pub))
            Error_Handler();
    if(!nh.advertise(actual_speed_pub))
            Error_Handler();
    if(!nh.advertise(obstacleDistance_pub))
            Error_Handler();

    if(!nh.subscribe(targetSpeed_sub))
        Error_Handler();
    if(!nh.subscribe(left_PID_sub))
        Error_Handler();
    if(!nh.subscribe(right_PID_sub))
        Error_Handler();

    // Never Changed
    double pcov[36] = { 0.1, 0.0, 0.0,   0.0,   0.0,   0.0,
                        0.0, 0.1, 0.0,   0.0,   0.0,   0.0,
                        0.0, 0.0, 1.0e6, 0.0,   0.0,   0.0,
                        0.0, 0.0, 0.0,   1.0e6, 0.0,   0.0,
                        0.0, 0.0, 0.0,   0.0,   1.0e6, 0.0,
                        0.0, 0.0, 0.0,   0.0,   0.0,   0.2};

    memcpy(&(odom.pose.covariance),  pcov, sizeof(double)*36);
    memcpy(&(odom.twist.covariance), pcov, sizeof(double)*36);
}


// Initializes the Global MCU specific package (MSP).
void
HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}


// The received LinearTarget speed is in m/s
// The received AngularTarget speed is in rad/s
static void
targetSpeed_cb(const geometry_msgs::Twist& speed) {
    double angSpeed  = speed.angular.z*TRACK_LENGTH*0.5; // Vt = Omega * R
    leftTargetSpeed  = speed.linear.x - angSpeed;        // in m/s
    rightTargetSpeed = speed.linear.x + angSpeed;        // in m/s
/// TODO:
/// Da spostare in Loop() per tenere conto del fatto che, se si interrompe
/// il collegamento con il Controller Remoto, Buggy deve FERMARSI !!!
    pLeftControlledMotor->setTargetSpeed(leftTargetSpeed);
    pRightControlledMotor->setTargetSpeed(rightTargetSpeed);
}


static void
left_PID_cb(const geometry_msgs::Vector3& msg) {
    pLeftControlledMotor->setPID(msg.x, msg.y, msg.z);
}


static void
right_PID_cb(const geometry_msgs::Vector3& msg) {
    pRightControlledMotor->setPID(msg.x, msg.y, msg.z);
}


// To handle the TIM2 (Periodic Sampler) interrupt.
void
TIM2_IRQHandler(void) {
    // Will call HAL_TIM_OC_DelayElapsedCallback(&hSamplingTimer)
    HAL_TIM_IRQHandler(&hSamplingTimer);
}


// To Restart Periodic Timers
void
HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == hSamplingTimer.Instance) {
        if(htim->Channel == AHRS_UPDATE_CHANNEL) { // Time to Update AHRS Data ? (400Hz)
            htim->Instance->CCR4 += AHRSSamplingPulses;
            if(bAHRSpresent) {
                Acc.get_Gxyz(AccelValues);
                Gyro.readGyro(GyroValues);
                Magn.ReadScaledAxis(MagValues);
                Madgwick.update(GyroValues, AccelValues, MagValues);
                isTimeToUpdateAHRS = true;
            }
            //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        else if(htim->Channel == MOTOR_UPDATE_CHANNEL) { // Time to Update Motors Data ? (50Hz)
            htim->Instance->CCR2 += motorSamplingPulses;
            if(pLeftControlledMotor) {
                pLeftControlledMotor->Update();
                isTimeToUpdateMotors = true;
            }
            if(pRightControlledMotor) {
                pRightControlledMotor->Update();
                isTimeToUpdateMotors = true;
            }
        }
        else if(htim->Channel == SONAR_UPDATE_CHANNEL) { // Time to Update Sonar Data ? (10Hz)
            htim->Instance->CCR3 += sonarSamplingPulses;
            uhCaptureIndex = 0;
            LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
            LL_TIM_EnableCounter(hSonarPulseTimer.Instance);
        }
    } // if(htim->Instance == hSamplingTimer.Instance)
}


// To handle the TIM5 (Sonar Echo) interrupt.
// Will call HAL_TIM_IC_CaptureCallback(&hSonarEchoTimer)
void
TIM5_IRQHandler(void) {
    HAL_TIM_IRQHandler(&hSonarEchoTimer);
}


// Called when the Sonar Echo Signal Change Level
void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(uhCaptureIndex == 0) { // Get the 1st Input Capture value
        // Select the next edge of the active transition on the TI2 channel: falling edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
        uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1) { // Get the 2nd Input Capture value
        // Select the next edge of the active transition on the TI2 channel: rising edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
        uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        /// Echo duration computation
        if(uwIC2Value2 >= uwIC2Value1) {
            uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
        }
        else if(uwIC2Value2 < uwIC2Value1) { // 0xFFFFFFFF is max TIM5_CCRx value
            uwDiffCapture = ((0xFFFFFFFF - uwIC2Value1) + uwIC2Value2) + 1;
        }
        uhCaptureIndex = 0;
        isTimeToUpdateSonar = true;
    }
}


// To handle the EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}


// EXTI line[15:10] interrupts handler
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) {

    }
}


// Tx Transfer completed callback
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    nh.getHardware()->flush();
}


// Rx Transfer completed callback
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    nh.getHardware()->reset_rbuf();
}


// To handle DMA TX interrupt request.
void
USART2_DMA_TX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmatx);
}


// To handle DMA RX interrupt request.
void
USART2_DMA_RX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmarx);
}


// To handle USART2 interrupt request.
void
USART2_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_UART_IRQHandler(&huart2);
}


void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    Error_Handler();
}


#ifdef  USE_FULL_ASSERT
char txBuffer[256];

void
assert_failed(uint8_t *file, uint32_t line) {
    sprintf((char *)txBuffer, "%s - line: %d", file, line);
    nh.loginfo(txBuffer);
    while(true) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);
    }
}
#endif // USE_FULL_ASSERT
