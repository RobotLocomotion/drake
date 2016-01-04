function paramEst_test
tmp = addpathTemporary(fullfile(pwd,'..'));

% Synthetic parameter estimation mode
% 'base':       Base case parameter estimation - intial params = true params
% 'paramerr':   w/ parameter error but no measurement noise
% 'measnoise':  w/ parameter error and w/ measurement noise
% 'delay':      w/ parameter error and w/ measurement noise and delay (Not complete)
mode = 'paramerr';

% Parameter Estimation model
% 'dynamic'     = use dynamic model - requires qdd
% 'energetic'   = use energetic model - doesn't require qdd
% 'simerr'      = use simulation error - doesn't require qdd
parameterEstimationOptions.model = 'dynamic';

% Method by which to obtain qdd (only used in dynamic model)
% 'manipul':   Use acrobot manipulator equations to estimate true qdd
% 'derivative': Take the derivative of qd
qddmode = 'derivative';

% Parameter Estimation method
% 'nonlinprog'  = nonlinear least squares (LS) to solve problem
% 'linprog'       = linear LS on lumped params then nonlinear LS to recover
%                 original parameters
% 'lsqnonlin'   = use MATLAB's built-in nonlinear least squares solver (debugging)
parameterEstimationOptions.method = 'nonlinprog';

% Option to print from estimator
% 'noprint'     = Do not print output from parameterEstimation.m, but will 
%                 still print output of paramEstSyntheticData.m script
% 'printEst'    = only print estimated from parameterEstimation.m
% 'printAll'	= print estimated and original from parameterEstimation.m
parameterEstimationOptions.print_result = 'noprint';

% Standard deviation of the data input error
if strcmp(parameterEstimationOptions.model,'energetic')
    % In this case, for theta1,theta2,theta1dot,theta2dot
    noisestd = sqrt([.0005, .0005, .0007, .0007]);
else
    % In this case, for theta1,theta2,theta1dot,theta2dot,theta1doubledot,theta2doubledot
    noisestd = sqrt([.0005, .0005, .0007, .0007, .0012, .0012,]);
end
% Standard deviation of the parameter value percent error
paramstd = 1/5;

r = AcrobotPlant;
rtrue = AcrobotPlant;
nq = r.num_positions;
% The symmetric positive definite cost matrix for simerr
% Err=sum((xobs(n)-xhat(n))'*C*(xobs(n)-xhat(n)))
C = eye(2*nq);

if ~strcmp(mode,'base')
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

outputFrameNames = r.getOutputFrame.getCoordinateNames();

%% Test on swingup up data
[utraj,xtraj] = swingUpTrajectory(rtrue);
Ts = .01; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
tsamples = T0:Ts:Tf;
usamples = eval(utraj,tsamples)';

% % Produce observations through direct collocation simulation
% xsamples = eval(xtraj,tsamples)';
% Produce observations through forward euler method
xsamples = computeTraj(rtrue,eval(xtraj,T0),usamples',tsamples)';

% FOR DEBUGGING TRAJECTORIES
% vtrue = AcrobotVisualizer(rtrue);
% vtrue.playback(xtraj);
% xtraj2pp = PPTrajectory(zoh(tsamples,xtraj2));
% xtraj2pp = xtraj2pp.setOutputFrame(xtraj.getOutputFrame);
% vtrue.playback(xtraj2pp);
% xtraj3 = computeTraj(r,xsamples(1,:)',usamples',tsamples);
% xtraj3pp = PPTrajectory(zoh(tsamples,xtraj3));
% xtraj3pp = xtraj3pp.setOutputFrame(xtraj.getOutputFrame);
% vtrue.playback(xtraj3pp);
% % figure; comet(xtraj2(1,:),xtraj2(2,:));
% % hold on; comet(xsamples(:,1)',xsamples(:,2)');
% figure; plot(xsamples(:,1));
% figure; plot(xsamples(:,2));
% figure; plot(xsamples(:,1)',xsamples(:,2)');
% hold on; plot(xtraj2(1,:),xtraj2(2,:));
% figure; plot(xsamples(:,1)',xsamples(:,2)');
% hold on; plot(xtraj2(1,:),xtraj2(2,:));
% plot(xtraj3(1,:),xtraj3(2,:));

%% Generate second derivative
if ~strcmp(parameterEstimationOptions.model,'energetic')
    if strcmp(qddmode,'manipul')
        qdd = zeros(length(tsamples),nq);
        for j=1:length(tsamples)
          [H,C,B] = manipulatorDynamics(rtrue,xsamples(j,1:nq)',xsamples(j,nq+(1:nq))');
          qdd(j,:) = (H\(B*usamples(j,:)' - C))';
        end
    else
        % Differentiating to get the second derivative of the state variables
        % TODO: Try lowpass filter
        qdd = deriv(xtraj,T0:Ts:Tf)'; qdd = qdd(:,nq+(1:nq));
    end
    xsamples = [xsamples qdd];
    outputFrameNames = [outputFrameNames;'theta1doubledot';'theta2doubledot'];
end

%% Add gaussian noise to measurements
if strcmp(mode,'measnoise') || strcmp(mode,'delay')
    measurementNoise = randn(size(xsamples))*diag(noisestd);
else
    measurementNoise = 0;
end
xsamplesfinal = xsamples+measurementNoise;
data = iddata(xsamplesfinal,usamples,Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);


%% Compute results from each model
coords = getCoordinateNames(r.getParamFrame);
p_true = double(rtrue.getParams);
p_init = double(r.getParams);

models = {'dynamic','simerr','energetic'};
mlen = length(models);
estimated_parameters = zeros(length(p_true),mlen);
simerror = zeros(1,mlen);
for i=1:mlen
    parameterEstimationOptions.model = models{i};
    [estimated_parameters(:,i),simerror(i)] = parameterEstimation(r,data,parameterEstimationOptions);
    %% Print out results
    fprintf('\nParameter estimation results for %s:\n\n',models{i});
    fprintf('  Param  \tTrue    \t Initial\tEstimated\n');
    fprintf('  -----  \t--------\t--------\t---------\n');
    for j=1:length(coords)
      fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\n',coords{j},p_true(j),p_init(j),estimated_parameters(j,i));
    end
end
disp(simerror)
end

function xtraj = computeTraj(obj,x0,utraj,t)
    nx = length(x0);
    dt = diff(t);
    N = length(dt);
    xtraj = [x0,zeros(nx,N)];
    for i=1:N
        f = dynamics(obj,t(i),xtraj(:,i),utraj(:,i));
        xtraj(:,i+1) = xtraj(:,i)+f*dt(i);
    end
end

%% Debugging purposes

% function simerr = computeErr(obj,p,xobs,utraj,t,O)
%     newobj = obj.setParams(p);
%     x0 = xobs(:,1);
%     xtraj = computeTraj(newobj,x0,utraj,t);
%     xdiff = xtraj - xobs;
%     simerr = sum(sum(xdiff.*(O*xdiff),1),2);
% end
% 


%     qddTrue = qdd;
%     qdd = deriv(xtraj,T0:Ts:Tf)'; qdd = qdd(:,nq+(1:nq));
%     figure;plot(qddTrue(:,1),'-g'); hold on; plot(qdd(:,1),'-r')
%     title(['q_1 True Acceleration vs. Derivative of Velocity']);
%     xlabel('Sample Time')
%     ylabel({'Acceleration Magnitude' '(m/s)/sample'})
%     legend('True Acceleration','Derivative of Vel');

% v = AcrobotVisualizer(r);
% vtrue = AcrobotVisualizer(rtrue);
% vtrue.playback(xtraj);
% 
% % plot(xsamples(:,1)); hold on; plot(xsamplesfinal(:,1));
% plot(xsamples); hold on; plot(xsamplesfinal);
% % Testing to see if estimate is produced by local minima
% r = rtrue;
