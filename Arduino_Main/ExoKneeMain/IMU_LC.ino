// wired into pins above AREF on Arduino MEGA

void setupIMUL() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C (LC) device...");
    IMU_LC.initialize();

    // verify connection
    Serial.println("Testing device (LC) connection...");
    Serial.println(IMU_LC.testConnection() ? "MPU6050 LC connection successful" : "MPU6050 LC connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal LC sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(IMU_LC.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_LC.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_LC.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_LC.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_LC.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_LC.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    IMU_LC.setXGyroOffset(220);
    IMU_LC.setYGyroOffset(76);
    IMU_LC.setZGyroOffset(-85);
    Serial.print(IMU_LC.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_LC.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_LC.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_LC.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_LC.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_LC.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
}

void loopIMUL() {
    // read raw accel/gyro measurements from device
    IMU_LC.getMotion6(&IMU_LC_ax, &IMU_LC_ay, &IMU_LC_az, &IMU_LC_gx, &IMU_LC_gy, &IMU_LC_gz);

    // these methods (and a few others) are also available
    //IMU_L.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
    //IMU_L.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

    #ifdef OUTPUT_READABLE_IMU_LC
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(IMU_LC_ax/18000); Serial.print("\t");
        Serial.print(IMU_LC_ay/18000); Serial.print("\t");
        Serial.print(IMU_LC_az/18000); Serial.print("\t");
        Serial.print(IMU_LC_gx); Serial.print("\t");
        Serial.print(IMU_LC_gy); Serial.print("\t");
        Serial.println(IMU_LC_gz);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

