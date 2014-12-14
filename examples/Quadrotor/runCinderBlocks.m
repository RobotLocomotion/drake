function [r, xtraj, utraj, prog] = runCinderBlocks
% simple program to build a simulation environment with cinder blocks

r = Quadrotor()
%r = addRobotFromURDF(r, 'cinder_blocks.urdf');
r = addRobotFromSDF(r, '/home/scottviteri/Downloads/drake-minimal-0.9.2-7d1238b-linux/drake/examples/Quadrotor/box2.world');
%v = constructVisualizer(r)

return;