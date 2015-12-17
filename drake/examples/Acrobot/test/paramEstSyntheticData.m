function paramEstSyntheticData

% Synthetic parameter estimation mode
% 0: Base case parameter estimation - intial params = true params
% 1: Try parameter estimation without any noise
% 2: Try parameter estimation with measurement noise
% 3: Try parameter estimation with noise and unknown delay (Not complete)
mode = 2;

% Method by which to obtain qdd
% 0: Use acrobot dynamics to estimate true qdd
% 1: Take the derivative of qd
qddmode = 0;

% Option to print from estimator: 0=noprint, 1=print
% Option 0 will still print output of script
parameterEstimationOptions.print_result = 0;

% Standard deviation of the data input error
% In this case, for theta1,theta2,theta1dot,theta2dot,theta1doubledot,theta2doubledot
noisestd = sqrt([.0005, .0005, .0007, .0007, .0012, .0012,]);
% Standard deviation of the parameter value percent error
paramstd = 1/5;

r = AcrobotPlant;
rtrue = AcrobotPlant;

if mode ~= 0
    % Perturb original parameter estimates with random percentage error
    % normally distributed with standard dev = paramstd, and greater than -1
    paramerr = randn(1,10)*paramstd;
    while sum(paramerr<=-1)~=0
        paramerr(paramerr<-1) = randn(1,sum(paramerr<-1))*paramstd;
    end

    % rtrue.l1 = rtrue.l1 + rtrue.l1*paramerr(1); 
    % rtrue.l2 = rtrue.l2 + rtrue.l2*paramerr(2); 
    % rtrue.m1 = rtrue.m1 + rtrue.m1*paramerr(3); 
    % rtrue.m2 = rtrue.m2 + rtrue.m2*paramerr(4);
    rtrue.b1  = rtrue.b1 + rtrue.b1*paramerr(5);
    rtrue.b2  = rtrue.b2 + rtrue.b2*paramerr(6);
    rtrue.lc1 = rtrue.lc1 + rtrue.lc1*paramerr(7); 
    rtrue.lc2 = rtrue.lc2 + rtrue.lc2*paramerr(8); 
    rtrue.Ic1 = rtrue.Ic1 + rtrue.Ic1*paramerr(9);  
    rtrue.Ic2 = rtrue.Ic2 + rtrue.Ic2*paramerr(10);
end

%% Test on swingup up data
[utraj,xtraj] = swingUpTrajectory(rtrue);
Ts = .01; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
tsamples = T0:Ts:Tf;
xsamples = eval(xtraj,tsamples)';
usamples = eval(utraj,tsamples)';

%% Generate second derivative
nq = r.num_positions;
if qddmode == 0
    qdd = zeros(length(tsamples),nq);
    for i=1:length(tsamples)
      [H,C,B] = manipulatorDynamics(rtrue,xsamples(i,1:nq)',xsamples(i,nq+(1:nq))');
      qdd(i,:) = (H\(B*usamples(i,:)' - C))';
    end
else
    % Differentiating to get the second derivative of the state variables
    % TODO: Try lowpass filter
    qdd = deriv(xtraj,T0:Ts:Tf)'; qdd = qdd(:,nq+(1:nq));
end
xsamples = [xsamples qdd];

%% Add gaussian noise to measurements
xsamplesfinal = xsamples;
if mode == 2
    xsamplesfinal = xsamples+randn(size(xsamples))*diag(noisestd);
end

% % Debugging purposes
% v = AcrobotVisualizer(r);
% vtrue = AcrobotVisualizer(rtrue);
% vtrue.playback(xtraj);
% 
% % plot(xsamples(:,1)); hold on; plot(xsamplesfinal(:,1));
% plot(xsamples); hold on; plot(xsamplesfinal);
% % Testing to see if estimate is produced by local minima
% r = rtrue;

outputFrameNames = [r.getOutputFrame.getCoordinateNames(); 'theta1doubledot'; 'theta2doubledot'];
data = iddata(xsamplesfinal,usamples,Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);
[estimated_parameters, estimated_delay] = parameterEstimation(r,rtrue,data,parameterEstimationOptions);

%% Print out results
coords = getCoordinateNames(r.getParamFrame);
p_true = double(rtrue.getParams);
p_init = double(r.getParams);
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \tTrue    \t Initial\tEstimated\n');
fprintf('  -----  \t--------\t--------\t---------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\n',coords{i},p_true(i),p_init(i),estimated_parameters(i));
end
