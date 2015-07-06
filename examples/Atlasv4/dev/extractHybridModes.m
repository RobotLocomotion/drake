function [ts,modes] = extractHybridModes(r,xtraj,switch_times)
% specific to Atlas. 
%   reasonable subset of possible modes:
%
%   mode 1: [left: heel+toe, right: heel+toe]
%   mode 2: [left: heel,     right: heel+toe]
%   mode 3: [left: toe,      right: heel+toe]
%   mode 4: [left: none,     right: heel+toe]
%   mode 5: [left: heel,     right: toe]
%   mode 6: [left: heel+toe, right: heel]
%   mode 7: [left: heel+toe, right: toe]
%   mode 8: [left: heel+toe, right: none]
%   mode 9: [left: toe,      right: heel]
%   mode 10: [left: none,    right: none]

if nargin < 3
  ts = linspace(xtraj.tspan(1),xtraj.tspan(2),1000);
%   ts = xtraj.getBreaks();
else
  ts = switch_times;
end

modes = [];

l_foot = findLinkId(r,'l_foot');
r_foot = findLinkId(r,'r_foot');

contact_options = struct('terrain_only',1,'body_idx',[1,l_foot,r_foot], ...
  'collision_groups',{{{'terrain'},{'heel','toe'},{'heel','toe'}}});
for i=1:length(ts)
  x = xtraj.eval(ts(i));
  phi = contactConstraints(r,x(1:r.getNumPositions),false, contact_options);
  
  in_contact = phi < 5e-3;
  
  if all(in_contact == [1;1;1;1])      
    modes = [modes,1];
  elseif all(in_contact == [1;0;1;1])
    modes = [modes,2];
  elseif all(in_contact == [0;1;1;1])
    modes = [modes,3];
  elseif all(in_contact == [0;0;1;1])
    modes = [modes,4];
  elseif all(in_contact == [1;0;0;1])
    modes = [modes,5];
  elseif all(in_contact == [1;1;1;0])
    modes = [modes,6];
  elseif all(in_contact == [1;1;0;1])
    modes = [modes,7];
  elseif all(in_contact == [1;1;0;0])
    modes = [modes,8];
  elseif all(in_contact == [0;1;1;0])
    modes = [modes,9];
  elseif ~any(in_contact)
    modes = [modes,10];
  else
    error('unknown mode');
  end
  
end


