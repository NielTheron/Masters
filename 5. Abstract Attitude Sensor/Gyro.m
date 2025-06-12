function GYR = Gyro(angelarVelocity,Gyr_noise)

GYR = angelarVelocity + Gyr_noise*randn(3,1);

end

