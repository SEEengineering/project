

#include "I2Cdev.h" // the I2C library for the I²C communiation with the MPU6050 (the (Accelerometer + Gyroscope)sensor) this is used when using the STM32 board

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"  // this is also an I²C library for communication with the MPU6050 , but this one is for the Arduino boards
#endif             // you see it's conditionally used (if we're using an arduino board)

MPU6050 mpu;
vr Vr; 

#define INTERRUPT_PIN PA1
// set these to correct for drift - try small values ~0.000001
#define ALPHA 0.f // correction for drift - tilt channel
#define BETA  0.f // correction for drift - yaw channel

// IMU status and control:
bool dmpReady = false;  // true if DMP init was successful
uint8_t mpuIntStatus;   
uint8_t devStatus;      // 0 = success, !0 = error
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 

Quaternion q;           // [w, x, y, z]  

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.setSDA(PB9);
        Wire.setSCL(PB8);
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Vr.start(); // serial.begin() starting the serial communiation
    
    mpu.initialize(); // initializing the mpu sensor by initializing (the clocksource + the gyroscope range + the accelerometer range + waking it up by disableing the sleep mode )
    pinMode(INTERRUPT_PIN, INPUT);
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //all the instructions inside the serial.println() means, if the mp.testConnection() returns a true , 
    //all these instructions will be replaced by the 1st option : F("MPU6050 connection successful")
    //else if it retuns a false , then it will be replaced by the second one : F("MPU6050 connection failed")
    //for short : we just test the mpu (mpu6050 sensor) connection througth the I²C . and we report that to the serial  !


    // configure the DMP
    // DMP (Digital Motion Processor)responsible for fusing the accelerometer and gyroscope data together 
    // it takes all the sensor data (gyroscope data + accelerometer data) and combine it in one data accessible via the  I²c  
    devStatus = mpu.dmpInitialize();

    // ==================================
    // supply your own gyro offsets here:
    // ==================================
    // follow procedure here to calibrate offsets https://github.com/kkpoon/CalibrateMPU6050
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);

    mpu.setZAccelOffset(1788);

    // devStatus if everything worked properly
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
    }
}

void loop() {
    // Do nothing if DMP doesn't initialize correctly
    if (!dmpReady) return; // a return will exit the code and start the loop over again

    // wait for MPU interrupt or extra packet(s) to be available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // an empty while means we are waiting for the condition to be flase means a mpuIntrupt to be true and the fifoCount>= packetSize
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    // getting the status of the dmp
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // we reset the dmp fifo(in which the sensor data been saved to)
        // if the reported status*0x10 ==0 or the fifocount reaches 1024 (like a max)
        // this fifo is used to hold the sensor data tell we read it 
        mpu.resetFIFO();
    }

    // check for interrupt
    else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;


       // where inputing in this funtion  &q ( pointer to the w,x,y,z (q) variables), 
       // and the fifobuffer , which is the sensor data are gotten throught the I²C
       // in this function where filling out the &q(which is the real data needed )
       // with the fifoBuffer data gotten from the sensor
       // the funstion uses pointers, means that we're updating these variable right in their RAM memory addresses
        mpu.dmpGetQuaternion(&q, fifoBuffer); //(1) in the document
    

        // find direction of gravity (down in world frame)
        VectorFloat down; // adding another pointer structure for the gravity data variables 

        mpu.dmpGetGravity(&down, &q); // (2) in the document
        
        // tilt angle is the angle made between world-y and the "up"-vector (just negative of "down")
        // the Magnetude = sqrt(x²+y²+z²)
        // but here where refering to the &down pointer we just have calculated which is the vector float
        // down.y is reffering to Vy . and down.getMagnitude() will take the Vx,Vy,Vz and inject them 
        // in the "sqrt(x²+y²+z²)"" . so down.getMagnitude() will be  sqrt(Vx²+Vy²+Vz²)
        // for the tilt angle :
        // ex. cos(@)=adj/hyp .  the hyp will definetely be the Magnetude
        // for the adj I'm not sure, i don't have an idea on the axises orientations and the object position they are reffering to 
        // here they state that the -Vy is the adj
        // so cos(phi)=-Vy/Magnetude   --->   phi = acos(-Vy/Magnetude )

        float phi = acos( -down.y / down.getMagnitude() );
        

        // drift due to tilt
        // another pointer structure d_tilt that will calculate the d_tilt for each q(w,x,y,z)
        //               (   d_tilt.w  ,         d_tilt.x                   ,  d_tilty ,               d_tilt.z             )   
        Quaternion d_tilt( cos(phi/2.f), -down.getNormalized().z * sin(phi/2.f), 0.f, down.getNormalized().x * sin(phi/2.f) );
        // ex. for the d_tilt.x it will be  -down.getNormalized().z * sin(phi/2.f)
        // down.getNormalized() will just take the Vector data (Vx,Vy,Vz) and divide them by the Magnetude
        // so each Vx,Vy,Vz will be replaced by Vx/m ,Vy/m ,Vz/m    (m=Magnetude)


        // TO DO: if magnetometer readings are available, yaw drift can be accounted for
        // int16_t ax_tmp, ay_tmp, az_tmp, gx_tmp, gy_tmp, gz_tmp, mx, my, mz;
        // mpu.getMotion9(ax_tmp, ay_tmp, az_tmp, gx_tmp, gy_tmp, gz_tmp, mx, my, mz);
        // VectorFloat16 magn(float(mx), float(my), float(mz));
        // VectorFloat16 north = magn.rotate(q);
        // float beta = acos( -north.z / north.magnitude() );
        // Quaternion d_yaw( cos(beta/2.f), 0.f, sin(beta/2.f), 0.f );
        Quaternion d_yaw; // just default to identity if no magnetometer is available.
         //   w = 1.0f;
         //   x = 0.0f;
         //   y = 0.0f;
         //   z = 0.0f;
        // COMPLEMENTARY FILTER (YAW CORRECTION * TILT CORRECTION * QUATERNION FROM ABOVE)
        // tilt_correct is calculated as  -ALPHA * 2.f * acos(d_tilt.w);
        // ALPHA (which is in the top on the code initialise as 0)
        // d_tilt.w = cos(phi/2.f)
        double tilt_correct = -ALPHA * 2.f * acos(d_tilt.w);
        /// same thing for the yaw_correct (BETA is also initialised to 0 in the top of the code )
        double yaw_correct  = -BETA  * 2.f * acos(d_yaw.w);
        // another pointer structure for clculating the tilt correction for each q(w,x,y,z)
         //                           tilt_correction.w  ,                              tilt_correction.x        ,tilt_correction.y,          tilt_correction.z
        Quaternion tilt_correction( cos(tilt_correct/2.f), -down.getNormalized().getMagnitude() * sin(tilt_correct/2.f), 0.f, down.getNormalized().x * sin(tilt_correct/2.f) );
        // samehting for the yaw correction
        //                          yaw_correction.w, yaw_correction.x ,yaw_correction.y, yaw_correction.z
        Quaternion yaw_correction( cos(yaw_correct/2.f), 0.f, sin(yaw_correct/2.f), 0.f);
        // applying the corrections to the Quaternions
        // qc = yc * tc * q  // q_correction = yaw_correction * tilt_correction * q
        // qc(w,x,y,z) = yc(w,x,y,z)*tc(w,x,y,z)*q(w,x,y,z)
        Quaternion qc(tilt_correction.w, tilt_correction.x, tilt_correction.y, tilt_correction.z);
       
        qc.getProduct(yaw_correction); // (3) in the document
        qc.getProduct(tilt_correction); //
        qc.getProduct(q); //

        // report result
        vr.updateOrientation(q.x, q.y, q.z, q.w, 4);  //(4) in the document
        
    }
}
