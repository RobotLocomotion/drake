function [sin_min,sin_max,cos_min,cos_max] = rangeOfSinCos(theta_min,theta_max)
% return the range of sin(theta) and cos(theta) if theta is within
% [theta_min theta_max]
assert(numel(theta_min) == 1 && numel(theta_max) == 1 && theta_min <= theta_max);
if(theta_max-theta_min>=pi*2)
  sin_min = -1;
  sin_max = 1;
  cos_min = -1;
  cos_max = 1;
else
  % move theta_min to [-2*pi 0];
  theta_change = ceil(theta_min/(2*pi))*2*pi;
  theta_min = theta_min-theta_change;
  theta_max = theta_max-theta_change;
  sin_max = -inf;
  sin_min = inf;
  cos_max = -inf;
  cos_min = inf;
  if(theta_min<=-1.5*pi && theta_max>-1.5*pi)
    sin_max = 1;
  elseif(theta_min<=-0.5*pi && theta_max>-0.5*pi)
    sin_min = -1;
  elseif(theta_min<=0.5*pi && theta_max>0.5*pi)
    sin_max = 1;
  elseif(theta_min<=1.5*pi && theta_max>1.5*pi)
    sin_min = -1;
  end
  sin_max = max([sin_max sin(theta_min) sin(theta_max)]);
  sin_min = min([sin_min sin(theta_min) sin(theta_max)]);
  if(theta_min<=-pi && theta_max > -pi)
    cos_min = -1;
  elseif(theta_min<=0 && theta_max > 0)
    cos_max = 1;
  elseif(theta_min<= pi && theta_max >pi)
    cos_min = -1;
  end
  cos_max = max([cos_max cos(theta_min) cos(theta_max)]);
  cos_min = min([cos_min cos(theta_min) cos(theta_max)]);
end