%Script IMU_FreeRTOS
clear all;
close all;
delete(instrfindall);

%Configure constants for test:
tTest       = 10; %segundos
tTaskSample = 20; %ms
QAngle      = 0.001;
QBias       = 0.003;
RMeasure    = 0.001;
alpha       = 0.7143; %0.98;

%Initialize Variables for Sample:
sample = (tTest*1000)/tTaskSample;
acelHigh = zeros(sample,3);
acelLow = zeros(sample,3);
gyroHigh = zeros(sample,3);
gyroLow = zeros(sample,3);
angle = zeros(sample,3);
encoder = zeros(sample,1);

%Open Serial Port:
s = serial('COM5','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 3, 'Terminator', 'CR/LF');
fclose(s);
fopen(s);
disp('Serial Port Opened');

%Parse command strings:
str_tTest       = sprintf('tTotalSample;1;%.3f',tTest);
str_tTaskSample = sprintf('tTaskSample;1;%.3f',tTaskSample);
str_QAngle      = sprintf('kalQAngle;1;%.3f',QAngle);
str_QBias       = sprintf('kalQBias;1;%.3f',QBias);
str_RMeasure    = sprintf('kalRMeasure;1;%.3f',RMeasure);
str_Alpha       = sprintf('alphaCFilter;1;%.3f',alpha);

%Send commands:
disp('Sending Configuration Commands');
fprintf(s,'%s\r',str_tTest);
fscanf(s);
fprintf(s,'%s\r',str_tTaskSample);
fscanf(s);
fprintf(s,'%s\r',str_QAngle);
fscanf(s);
fprintf(s,'%s\r',str_QBias);
fscanf(s);
fprintf(s,'%s\r',str_RMeasure);
fscanf(s);
fprintf(s,'%s\r',str_Alpha);
fscanf(s);

%Start Command:
fprintf(s,'%s\r','goReset');

disp('IMU Read Started!');

%Sample Values:
i = 0;
while true
    out = fscanf(s);
    t = strcmp(out(1:4),'STOP');
    if t == 1
        break;
    end
    strVal = strsplit(out, ';');
    
    i = i+1;
    
    k = 1;
    acelLow(i,1) = str2double(strVal(k));
    k= k+1;
    acelLow(i,2) = str2double(strVal(k));
    k= k+1;
    acelLow(i,3) = str2double(strVal(k));
    k= k+1;
    
    acelHigh(i,1) = str2double(strVal(k));
    k= k+1;
    acelHigh(i,2) = str2double(strVal(k));
    k= k+1;
    acelHigh(i,3) = str2double(strVal(k));
    k= k+1;
    
    gyroLow(i,1) = str2double(strVal(k));
    k= k+1;
    gyroLow(i,2) = str2double(strVal(k));
    k= k+1;
    gyroLow(i,3) = str2double(strVal(k));
    k= k+1;
    
    gyroHigh(i,1) = str2double(strVal(k));
    k= k+1;
    gyroHigh(i,2) = str2double(strVal(k));
    k= k+1;
    gyroHigh(i,3) = str2double(strVal(k));
    k= k+1;
    
    angle(i,1) = str2double(strVal(k));
    k= k+1;
    angle(i,2) = str2double(strVal(k));
    k= k+1;
    angle(i,3) = str2double(strVal(k));
    
    k= k+1;
    encoder(i) = str2double(strVal(k));
end

fclose(s);
delete(s);

disp('IMU Read Finished');

%Tempo = (0:tTaskSample/1000:tTest)';
%Tempo = linspace(tTaskSample/1000,tTest,i);
%Tempo = linspace(0,tTest,i);
Tempo = linspace(0,tTest,length(angle(:,1)));
%Tempo = (0:tTaskSample/1000:( (length(angle(:,1))-1) * tTaskSample/1000));

figure;
plot(Tempo, angle(:,1), '-*');
hold on;
plot(Tempo, angle(:,2), '-*r');
hold on;
plot(Tempo, angle(:,3), '-*k');
hold on;
legend('Pure Angle', 'Compl. Angle', 'Kalman Angle');
title('Angle');
grid on;

figure;
plot(Tempo, acelLow(:,1), '-*');
hold on;
plot(Tempo, acelLow(:,2), '-*r');
hold on;
plot(Tempo, acelLow(:,3), '-*k');
hold on;
plot(Tempo, acelHigh(:,1), '-s');
hold on;
plot(Tempo, acelHigh(:,2), '-sr');
hold on;
plot(Tempo, acelHigh(:,3), '-sk');
hold on;
legend('X_L_o_w', 'Y_L_o_w', 'Z_L_o_w', 'X_H_i_g_h', 'Y_H_i_g_h', 'Z_H_i_g_h');
title('Acel (mG)');
grid on;

figure;
plot(Tempo, gyroLow(:,3), '-*k');
hold on;
plot(Tempo, gyroHigh(:,3), '-*');
hold on;
legend('Z_L_o_w', 'Z_H_i_g_h');
title('Gyro (º/s)');
grid on;

% X_mean = mean(acelLow(:,1));
% disp('X_mean:');
% disp(X_mean);
% Y_mean = mean(acelLow(:,2));
% disp('Y_mean:');
% disp(Y_mean);
% Z_mean = mean(acelLow(:,3));
% disp('Z_mean:');
% disp(Z_mean);
% finalVetor = sqrt(X_mean^2 + Y_mean^2 + Z_mean^2);
% disp('Result Vector:');
% disp(finalVetor);
% 
% Zgyro_mean = mean(gyroLow(:,3));
% disp('Zgyro_mean:');
% disp(Zgyro_mean);