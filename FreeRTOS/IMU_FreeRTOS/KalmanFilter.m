%Kalman Filter:

dt = 0.02;

A = [ 1 -dt;
      0   1];
B = [ dt; 0 ];
C = [ 1 0 ];
D = 0;

Plant = ss(A,B,C,D,-1,'inputname','u','outputname','y');

Q = [0.001 0.003];
Q = 0.001;
R = 0.03;

[kalmf,L,~,M,Z] = kalman(Plant,Q,R);

kalmf = kalmf(1,:);