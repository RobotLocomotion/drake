function testManipulatorEquations

path = addpathTemporary(fullfile(pwd,'..'));
plant=AcrobotPlant();
[H,C_times_v,G,B] = plant.manipulatorEquations()
