function runValueIteration

plant = PendulumPlant;
mdp = MarkovDecisionProcess.discretizeSystem(plant,{[-pi:.2:pi],[-4:.2:4]},linspace(-1,1,9),1e-2);

