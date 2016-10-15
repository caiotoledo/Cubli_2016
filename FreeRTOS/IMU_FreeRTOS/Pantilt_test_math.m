clear;
close all;

%Set Sine Period
Tp =    3;
Ts =    0.02;
Iteration = 4;

[tilt_steps, angle_steps, Tempo] = SineTiltGenerate(Ts,Tp,Iteration);

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