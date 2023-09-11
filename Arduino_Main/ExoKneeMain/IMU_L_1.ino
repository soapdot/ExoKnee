// wired into pins above AREF on Arduino MEGA

void setupIMUL() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println("Initializing I2C (L) device...");
    IMU_L.initialize();

    // verify connection
    Serial.println("Testing device (L) connection...");
    Serial.println(IMU_L.testConnection() ? "MPU6050 L connection successful" : "MPU6050 L connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(IMU_L.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_L.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_L.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_L.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_L.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_L.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    IMU_L.setXGyroOffset(220);
    IMU_L.setYGyroOffset(76);
    IMU_L.setZGyroOffset(-85);
    Serial.print(IMU_L.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(IMU_L.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(IMU_L.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(IMU_L.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_L.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(IMU_L.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
}

void loopIMUL() {
    // read raw accel/gyro measurements from device
    IMU_L.getMotion6(&IMU_L_ax, &IMU_L_ay, &IMU_L_az, &IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

    // these methods (and a few others) are also available
    //IMU_L.getAcceleration(&IMU_L_ax, &IMU_L_ay, &IMU_L_az);
    //IMU_L.getRotation(&IMU_L_gx, &IMU_L_gy, &IMU_L_gz);

    #ifdef OUTPUT_READABLE_IMU_L
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(IMU_L_ax); Serial.print("\t");
        Serial.print(IMU_L_ay); Serial.print("\t");
        Serial.print(IMU_L_az); Serial.print("\t");
        Serial.print(IMU_L_gx); Serial.print("\t");
        Serial.print(IMU_L_gy); Serial.print("\t");
        Serial.println(IMU_L_gz);
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

