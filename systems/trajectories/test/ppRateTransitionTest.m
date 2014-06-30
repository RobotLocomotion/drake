function ppRateTransitionTest

pp = foh(0:4,randn(1,5));

rateblock = PPRateTransition(4,2,1);
sys = cascade(setOutputFrame(ConstantTrajectory([pp.breaks(:);pp.coefs(:)]),getInputFrame(rateblock)),rateblock);

ytraj = simulate(sys,[0,5]);
fnplt(ytraj);

ytraj_desired = PPTrajectory(pp);
valuecheck(ytraj,ytraj_desired);


