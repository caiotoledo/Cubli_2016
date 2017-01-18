function [angleKalman] = kalmanFilter (angle, gyro, dt, P, bias, Qangle, Qgyrobias, Rmeasure)

angleKalman = zeros(1,length(angle));
angleKalman(1) = angle(1);

for n = 2:length(angle)
    rate = gyro(n) - bias;
    angleKalman(n) = angleKalman(n-1) + dt*rate;

    P(1,1) = P(1,1) + dt * (dt*P(2,2) - P(1,2) - P(2,1) + Qangle);
    P(1,2) = P(1,2) - P(2,2);
    P(2,1) = P(2,1) - P(2,2);
    P(2,2) = P(2,2) + Qgyrobias * dt;
    
    S = P(1,1) + Rmeasure;
    
    K(1) = P(1,1)/S;
    K(2) = P(2,1)/S;
    
    y = angle(n) - angleKalman(n);
    
    angleKalman(n) = angleKalman(n) + K(1) * y;
    bias = bias + K(2) * y;
    
    P00_temp = P(1,1);
	P01_temp = P(1,2);
    
    P(1,1) = P(1,1) - K(1) * P00_temp;
    P(1,2) = P(1,2) - K(1) * P01_temp;
    P(2,1) = P(2,1) - K(2) * P00_temp;
    P(2,2) = P(2,2) - K(2) * P01_temp;
end

end