% a = arduino()
% imu1 = lsm9ds1(a,'I2CAddress',[0x6B 0x1E])
% imu2 = lsm9ds1(a,'I2CAddress',[0x6A 0x1C])

clear a1 a2 g1 g2 m1 m2; close all
i = 0;
j = 0;
while true
    a1(i+1,:) = readAcceleration(imu1);
    g1(i+1,:) = readAngularVelocity(imu1);
    m1(i+1,:) = readMagneticField(imu1);
    figure(1)
    subplot(3,1,1); plot(a1); title('IMU1 Acceleration')
    subplot(3,1,2); plot(g1); title('IMU1 Angular Velocity')
    subplot(3,1,3); plot(m1); title('IMU1 Magnetic Field')
    
    a2(i+1,:) = readAcceleration(imu2);
    g2(i+1,:) = readAngularVelocity(imu2);
    m2(i+1,:) = readMagneticField(imu2);
    figure(2)
    subplot(3,1,1); plot(a2); title('IMU2 Acceleration')
    subplot(3,1,2); plot(g2); title('IMU2 Angular Velocity')
    subplot(3,1,3); plot(m2); title('IMU2 Magnetic Field')
   
    i = i + 1;
    j = j + 1;
end