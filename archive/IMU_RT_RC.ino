/* ==============================================
ExoSuit Project Main: 
IMU (inertial measurement unit) Sensor for Right Thigh & Calf
Wired to: I2C pins "SDA/SCL" on Arduino MEGA
IMU Sensor for Left Calf
Wired to: I2C pins above AREF on Arduino MEGA
=================================================
For IMUs; x, y, z defined as:
. x forward (pointing past knee), 
. y up (pointing toward body of user), 
. z away from leg (toward observer)
when looking at the leg from the side view 
=================================================*/

void setupIMUR() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C (R) devices...");
    IMU_RT.initialize();
    IMU_RC.initialize();

    // verify connection
    Serial.println("Testing device (R) connections...");
    Serial.println(IMU_RT.testConnection() ? "MPU6050 RT connection successful" : "MPU6050 RT connection failed");
    Serial.println(IMU_RC.testConnection() ? "MPU6050 RC connection successful" : "MPU6050 RC connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal RT sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(IMU_RT.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_RT.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_RT.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_RT.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RT.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RT.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    IMU_RT.setXGyroOffset(220);
    IMU_RT.setYGyroOffset(76);
    IMU_RT.setZGyroOffset(-85);
    Serial.print(IMU_RT.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_RT.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_RT.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_RT.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RT.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RT.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal RC sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(IMU_RC.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_RC.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_RC.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_RC.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RC.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RC.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    IMU_RC.setXGyroOffset(220);
    IMU_RC.setYGyroOffset(76);
    IMU_RC.setZGyroOffset(-85);
    Serial.print(IMU_RC.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_RC.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_RC.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_RC.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RC.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_RC.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
}

void loopIMUR() {
    // read raw accel/gyro measurements from device
    IMU_RT.getMotion6(&IMU_RT_ax, &IMU_RT_ay, &IMU_RT_az, &IMU_RT_gx, &IMU_RT_gy, &IMU_RT_gz);
    IMU_RC.getMotion6(&IMU_RC_ax, &IMU_RC_ay, &IMU_RC_az, &IMU_RC_gx, &IMU_RC_gy, &IMU_RC_gz);

    // these methods (and a few others) are also available
    //IMU_RT.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
    //IMU_RT.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

    #ifdef OUTPUT_READABLE_IMU_RT
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(IMU_RT_ax); Serial.print("\t");
        Serial.print(IMU_RT_ay); Serial.print("\t");
        Serial.print(IMU_RT_az); Serial.print("\t");
        Serial.print(IMU_RT_gx); Serial.print("\t");
        Serial.print(IMU_RT_gy); Serial.print("\t");
        Serial.println(IMU_RT_gz);
    #endif

    #ifdef OUTPUT_READABLE_IMU_RC
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(IMU_RC_ax); Serial.print("\t");
        Serial.print(IMU_RC_ay); Serial.print("\t");
        Serial.print(IMU_RC_az); Serial.print("\t");
        Serial.print(IMU_RC_gx); Serial.print("\t");
        Serial.print(IMU_RC_gy); Serial.print("\t");
        Serial.println(IMU_RC_gz);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

