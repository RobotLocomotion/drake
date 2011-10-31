function  robot = planar2()

% planar2  quick 2-link planar robot (e.g. for simulink)
% planar2  returns a stored copy of the robot model planarN(2).  The copy is
% created upon the first call to planar2, and is stored in a persistent
% variable.

persistent memory;

if length(memory) == 0
  memory = planarN(2);
end

robot = memory;
