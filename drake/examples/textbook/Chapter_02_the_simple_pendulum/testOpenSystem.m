function testOpenSystem

cd(fullfile(getDrakePath,'examples','Pendulum'));
open_system('constantTorqueDemo');
% hit play to start the simulation, and click on the boxes
% labeled as 'slider' to control the torque and damping

