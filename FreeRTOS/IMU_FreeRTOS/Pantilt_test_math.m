clear;
close all;

%Set Sine Period
Ts =    0.02;
Tp =    2;
Iteration = 10;
max_angle = 40;

[tilt_steps, angle_steps, Tempo] = SineTiltGenerate(max_angle,Ts,Tp,Iteration);

figure;
plot(Tempo, angle_steps, '-*');
grid on;
title('Tilt Angle Positions');

gyro_steps = diff(angle_steps);
gyro_steps = horzcat(gyro_steps(1),gyro_steps);
figure;
plot(Tempo, gyro_steps, '-*k');
grid on;
title('Tilt Gyro');