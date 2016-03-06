function  robot = planar3()

% planar3  quick 3-link planar robot (e.g. for simulink)
% planar3  returns a stored copy of the robot model planarN(3).  The copy is
% created upon the first call to planar3, and is stored in a persistent
% variable.

persistent memory;

if length(memory) == 0
  memory = planarN(3);
end

robot = memory;
