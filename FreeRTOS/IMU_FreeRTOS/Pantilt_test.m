%Set Sine Period
Tp =    3;
Ts =    0.02;
iteration = 5;
max_angle = 30;

[tilt_steps, angle_steps, Tempo] = SineTiltGenerate(max_angle,Ts,Tp,iteration);

%Open Serial Port:
s = serial('COM3','BaudRate', 9600, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 3, 'Terminator', 'CR/LF');
fclose(s);
fopen(s);

%Send initial position:
str_tiltpos = sprintf('TP%.0f',tilt_steps(1));
fprintf(s,'%s\r',str_tiltpos);
out = fscanf(s);
pause(1);

for i = 1:length(tilt_steps)
    
    str_tiltpos = sprintf('TP%.0f',tilt_steps(i));
    fprintf(s,'%s\r',str_tiltpos);
    out = fscanf(s);
    pause(Ts);
    
end

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