%Script IMU_FreeRTOS
clear;

tTest       = 10; %segundos
tTaskSample = 20; %ms
QAngle      = 0.001;
QBias       = 0.003;
RMeasure    = 0.03;

sample = (tTest*1000)/tTaskSample;
acel = zeros(sample,3);
gyro = zeros(sample,3);
angle = zeros(sample,3);

s = serial('COM3','BaudRate', 115200, 'DataBits', 8, 'StopBits', 1, 'Parity', 'none', 'Timeout', 3, 'Terminator', 'CR/LF');
fopen(s);

str_tTest       = sprintf('tTotalSample;1;%.3f',tTest);
str_tTaskSample = sprintf('tTaskSample;1;%.3f',tTaskSample);
str_QAngle      = sprintf('kalQAngle;1;%.3f',QAngle);
str_QBias       = sprintf('kalQBias;1;%.3f',QBias);
str_RMeasure    = sprintf('kalRMeasure;1;%.3f',RMeasure);

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

fprintf(s,'%s\r','goReset');

t = 0;
i = 1;
for n = 1:sample
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

    i = i +1;
end

fclose(s);
delete(s);