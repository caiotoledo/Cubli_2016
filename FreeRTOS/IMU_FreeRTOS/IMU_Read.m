%Script IMU_FreeRTOS
clear all;
close all;

%Configure constants for test:
tTest       = 10; %segundos
tTaskSample = 20; %ms
QAngle      = 0.001;
QBias       = 0.003;
RMeasure    = 0.03;
alpha       = 0.98; %0.7143;

%Initialize Variables for Sample:
sample = (tTest*1000)/tTaskSample;
acel = zeros(sample,3);
gyro = zeros(sample,3);
angle = zeros(sample,3);
encoder = zeros(sample,1);

%Open Serial Port:
s = serial('COM3','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 3, 'Terminator', 'CR/LF');
fclose(s);
fopen(s);

%Parse command strings:
str_tTest       = sprintf('tTotalSample;1;%.3f',tTest);
str_tTaskSample = sprintf('tTaskSample;1;%.3f',tTaskSample);
str_QAngle      = sprintf('kalQAngle;1;%.3f',QAngle);
str_QBias       = sprintf('kalQBias;1;%.3f',QBias);
str_RMeasure    = sprintf('kalRMeasure;1;%.3f',RMeasure);
str_Alpha       = sprintf('alphaCFilter;1;%.3f',alpha);

%Send commands:
fprintf(s,'%s\r',str_tTest);
out = fscanf(s);
fprintf(s,'%s\r',str_tTaskSample);
out = fscanf(s);
fprintf(s,'%s\r',str_QAngle);
out = fscanf(s);
fprintf(s,'%s\r',str_QBias);
out = fscanf(s);
fprintf(s,'%s\r',str_RMeasure);
out = fscanf(s);
fprintf(s,'%s\r',str_Alpha);
out = fscanf(s);

%Start Command:
fprintf(s,'%s\r','goReset');

%Sample Values:
for i = 1:sample
    out = fscanf(s);
    strVal = strsplit(out, ';');
    
    k = 1;
    acel(i,1) = str2double(strVal(k));
    k= k+1;
    acel(i,2) = str2double(strVal(k));
    k= k+1;
    acel(i,3) = str2double(strVal(k));
    k= k+1;
    
    gyro(i,1) = str2double(strVal(k));
    k= k+1;
    gyro(i,2) = str2double(strVal(k));
    k= k+1;
    gyro(i,3) = str2double(strVal(k));
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

Tempo = (0:tTaskSample/1000:(tTest-tTaskSample/1000))';

figure;
plot(Tempo, angle(:,1));
hold on;
plot(Tempo, angle(:,2), 'r');
hold on;
plot(Tempo, encoder, 'm');
hold on;
legend('Pure Angle', 'Compl. Angle', 'Kalman Angle', 'Encoder');
title('Angle');
grid on;

figure;
plot(Tempo, acel(:,1));
hold on;
plot(Tempo, acel(:,2), 'r');
hold on;
legend('X', 'Y');
title('Acel (mG)');
grid on;

figure;
plot(Tempo, gyro(:,3), 'k');
hold on;
legend('Z');
title('Gyro (º/s)');
grid on;

X_mean = mean(acel(:,1))
Y_mean = mean(acel(:,2))
Zgyro_mean = mean(gyro(:,3))