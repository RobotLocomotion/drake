function testCartPolePassive

cd(fullfile(getDrakePath,'examples','CartPole'));

plant=CartPolePlant();
[H,C_times_v,G,B] = plant.manipulatorEquations()

runPassive();  % simulate with force=0 (and display the results)

