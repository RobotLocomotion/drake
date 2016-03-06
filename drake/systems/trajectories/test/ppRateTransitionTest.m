function ppRateTransitionTest

pp = foh(0:4,randn(1,5));
ytraj_desired = PPTrajectory(pp);
ts = 0:.1:5;

rateblock = PPRateTransition(4,2,1);
pptraj = setOutputFrame(ConstantTrajectory([pp.breaks(:);pp.coefs(:)]),getInputFrame(rateblock));

sys = cascade(pptraj,rateblock);
ytraj = simulate(sys,[0,5]);
figure(1); subplot(2,1,1); fnplt(ytraj);
valuecheck(eval(ytraj,ts),eval(ytraj_desired,ts));

pptraj = setSampleTime(pptraj,[1;0]);
sys = cascade(pptraj,rateblock);
ytraj = simulate(sys,[0,5]);
figure(1); subplot(2,1,2); fnplt(ytraj);
valuecheck(eval(ytraj,ts),eval(ytraj_desired,ts));



