function paramEstSyntheticData

oldpath = addpath(fullfile(pwd,'..'));

r = AcrobotPlant;
v = AcrobotVisualizer(r);

true_parameters = getParams(r);

%% test on swingup up data
[utraj,xtraj] = swingUpTrajectory(r);

% Try parameter estimation without any noise
Ts = .01; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
data = iddata(eval(xtraj,T0:Ts:Tf)',eval(utraj,T0:Ts:Tf)',Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',r.getOutputFrame.getCoordinateNames());

estimated_parameters = parameterEstimation(r,data);


% Try parameter estimation with noise


% Try parameter estimation with noise and known delay


path(oldpath);