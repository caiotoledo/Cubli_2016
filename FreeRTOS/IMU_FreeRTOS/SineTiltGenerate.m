function [tilt_steps, angle_steps, Tempo] = SineTiltGenerate (Ts, Tp, iteration)

pos2angle =  0.05143;
max_pos =   596;

len =       Tp/Ts;
lenTotal =  iteration*len;
pi_steps =  linspace(-iteration*pi,iteration*pi,lenTotal);
seno =      sin(pi_steps);

tilt_steps = round(max_pos*seno);
angle_steps = tilt_steps*pos2angle;

Tempo = 0:Ts:(Tp*iteration-Ts);