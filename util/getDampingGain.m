function Kd = getDampingGain(Kp,damping_ratio)
%GETDAMPINGGAIN computes damping gain given proportunal gain and a desired
%damping ratio

Kd = 2* damping_ratio * sqrt(Kp);

end

