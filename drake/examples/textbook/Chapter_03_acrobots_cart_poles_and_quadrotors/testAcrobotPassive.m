function testAcrobotPassive

cd(fullfile(getDrakePath,'examples','Acrobot'));

plant=AcrobotPlant();
[H,C_times_v,G,B] = plant.manipulatorEquations()

runPassive();  % simulate with torque=0 (and display the results)

