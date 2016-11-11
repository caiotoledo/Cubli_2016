function [tilt_steps, angle_steps, Tempo, send_index] = SineTiltGenerate (maxAngle, Ts, Tp, iteration, res)

pos2angle =  0.0514;

max_pos = maxAngle/pos2angle;

if max_pos > 596
    max_pos = 596;
end

len =       Tp/Ts;
pi_steps = -pi*iteration:2*pi/len:pi*iteration;
seno =      cos(pi_steps);

tilt_steps = round(max_pos*seno);
angle_steps = tilt_steps*pos2angle;

Tempo = 0:Ts:(Tp*iteration);

%Indentify Sign Transition:
j = 1;
x = square(-pi*iteration*res:(2*pi/len)*res:pi*iteration*res)*max(angle_steps);
for i=1:length(x)-1
    if (x(i) ~= x(i+1))
        send_index(j) = i+1;
        j = j + 1;
    end
end