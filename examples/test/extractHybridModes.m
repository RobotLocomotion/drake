function [ts,modes] = extractHybridModes(r,xtraj,switch_times)
% specific to one-legged hopper. 
% 
%   mode 1: heel+toe
%   mode 2: toe
%   mode 3: heel
%   mode 4: flight

if nargin < 3
  ts = xtraj.tspan(1):0.001:xtraj.tspan(2);
%   ts = xtraj.getBreaks();
else
  ts = switch_times;
end

modes = [];

for i=1:length(ts)
  x = xtraj.eval(ts(i));
  phi = r.contactConstraints(x(1:r.getNumPositions));
  in_contact = phi < 5e-3;
  if in_contact(1) && in_contact(2) % heel+toe
    modes = [modes,1];
  elseif in_contact(1) % toe
    modes = [modes,2];
  elseif in_contact(2) % heel
    modes = [modes,3];
  else
    modes = [modes,4]; % flight
  end
end


