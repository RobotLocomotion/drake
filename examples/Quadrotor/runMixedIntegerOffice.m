function [r, xtraj, utraj, prog] = runMixedIntegerOffice
% simple program to build an office simulation environment


r = Quadrotor();
r = addRobotFromURDF(r, 'office.urdf');
v = constructVisualizer(r);

return;

