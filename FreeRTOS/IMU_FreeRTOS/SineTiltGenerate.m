function [tilt_steps, angle_steps, Tempo] = SineTiltGenerate (maxAngle, Ts, Tp, iteration)

pos2angle =  0.05143;

max_pos = maxAngle/pos2angle;

if max_pos > 596
    max_pos = 596;
end

len =       Tp/Ts;
lenTotal =  iteration*len;
pi_steps =  linspace(-iteration*pi,iteration*pi,lenTotal);
seno =      sin(pi_steps);

tilt_steps = round(max_pos*seno);
angle_steps = tilt_steps*pos2angle;

Tempo = 0:Ts:(Tp*iteration-Ts);