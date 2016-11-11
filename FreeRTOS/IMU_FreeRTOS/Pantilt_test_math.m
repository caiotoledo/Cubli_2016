clear;
close all;

%Set Sine Period
Tp =    2;
Ts =    0.02;
Iteration = 5;
max_angle = 5;

resolution = 1;

[tilt_steps, angle_steps, Tempo, tilt_index] = SineTiltGenerate(max_angle,Ts,Tp,Iteration,resolution);

angle_send = angle_steps(tilt_index);
Tempo_send = Tempo(tilt_index);

figure;
plot(Tempo, angle_steps, '-*');
hold on;
plot(Tempo_send, angle_send, '-*r');
grid on;
title('Tilt Angle Positions');

gyro_steps = diff(angle_steps);
% gyro_steps = horzcat(gyro_steps(1),gyro_steps);
figure;
plot(Tempo(2:length(Tempo)), gyro_steps, '-*k');
grid on;
title('Tilt Gyro');