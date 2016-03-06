function paramEst_test
% global PTRUE
tmp = addpathTemporary(fullfile(pwd,'..'));

parameterEstimationOptions.method = 'nonlinprog'; % nonlinear least squares
parameterEstimationOptions.C = eye(4); % The symmetric positive definite cost matrix for simerr
paramstd = 0.1; % Standard deviation of the parameter value percent error
noisestd = sqrt([.00025, .00025, .00035, .00035]); % Noise standard deviation for theta1,theta2,theta1dot,theta2dot

numTests = 20;
models = {'simerr','dynamic','energetic'};
% models = {'dynamic','energetic'};
mlen = length(models);

rtrue = AcrobotPlant;
p_true = double(rtrue.getParams);
% PTRUE = p_true;

%% Test on swingup up data
[utraj,xtraj] = swingUpTrajectory(rtrue);
Ts = .001; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
tsamples = T0:Ts:Tf;
usamples = eval(utraj,tsamples)';
xsamples = computeTraj(rtrue,eval(xtraj,T0),usamples',tsamples)'; % Produce state traj through forward euler method

r = AcrobotPlant;
nq = r.num_positions;
outputFrameNames = [r.getOutputFrame.getCoordinateNames();'theta1doubledot';'theta2doubledot'];
paramNames = getCoordinateNames(r.getParamFrame);
pmin = [0.02; 0.02; 0.1; 0.1; 0.05; 0.1;];
pmax = [1; 1; 2; 2; 1; 1;];
r = r.setParamLimits(pmin,pmax);

simerror = zeros(mlen,numTests);
simtime = zeros(mlen,numTests);
simflag = zeros(mlen,numTests);
p_init = zeros(length(p_true),numTests);
p_est = cell(1,mlen);
[p_est{:}] = deal(zeros(length(p_true),numTests));
for j=1:numTests
    %% Perturb original parameter estimates with random percentage error
    % normally distributed with standard dev = paramstd, and greater than -1
    paramerr = randn(1,6)*paramstd;
    while sum(paramerr<=-1)>0
        paramerr(paramerr<-1) = randn(1,sum(paramerr<-1))*paramstd;
    end
    r.b1  = r.b1 + r.b1*paramerr(1);
    r.b2  = r.b2 + r.b2*paramerr(2);
    r.lc1 = r.lc1 + r.lc1*paramerr(3); 
    r.lc2 = r.lc2 + r.lc2*paramerr(4); 
    r.Ic1 = r.Ic1 + r.Ic1*paramerr(5);  
    r.Ic2 = r.Ic2 + r.Ic2*paramerr(6);

    p_init(:,j) = double(r.getParams);
    
    %% Add gaussian noise to measurements
    measurementNoise = randn(size(xsamples))*diag(noisestd);
    xsamplesnoisy = xsamples+measurementNoise;

    %% Generate second derivative
    % Differentiating to get the second derivative of the state variables
    dtsamples = diff(tsamples)';
    qdd = diff(xsamplesnoisy(:,nq+(1:nq)),1,1)./repmat(dtsamples,1,nq);
    xsamplesfinal = [xsamplesnoisy(1:length(qdd),:), qdd];
    data = iddata(xsamplesfinal,usamples(1:length(qdd),:),Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);

    %% Compute results from each model    
    for i=1:mlen
        parameterEstimationOptions.model = models{i};
        tic;
        [p_est{i}(:,j),simerror(i,j),~,simflag(i,j)] = parameterEstimation(r,data,parameterEstimationOptions);
        simtime(i,j) = toc;
% %         DEBUGGING SNOPT FAILURES
%         if simflag(i,j)==41
%             fprintf('parameterEstimation failed\n');
%             tic;
%             [p_est{i}(:,j),simerror(i,j),~,simflag(i,j)] = parameterEstimation(r,data,parameterEstimationOptions);
%             simtime(i,j) = toc;
%         end
        
        fprintf('n: %i \tModel: %10s \tTime: %f \tSim Error: %f\n',j,models{i},simtime(i,j),simerror(i,j));
%         %% Print out results
%         fprintf('\nParameter estimation results for %s:\n\n',models{i});
%         fprintf('  Param  \tTrue    \t Initial\tEstimated\n');
%         fprintf('  -----  \t--------\t--------\t---------\n');
%         for k=1:length(paramNames)
%           fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\n',paramNames{k},p_true(k),p_init(k),p_est{i}(k,j));
%         end
    end
end
disp(simerror);
disp(simtime);
simerrInd = find(strcmp(models,'simerr')==1);
meanSimerror = mean(simerror,2);
meanSimerror(simerrInd) = mean(simerror(simerrInd,simflag(simerrInd,:)==1));
meanSimtime = mean(simtime,2);
meanSimtime(simerrInd) = mean(simtime(simerrInd,simflag(simerrInd,:)==1));
disp(meanSimerror);
disp(meanSimtime);
% save('data1_001','simerror','simtime','simflag','p_true','p_init','p_est');
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

%% DEBUGGING CODE
% qddTrue = qdd;
% qdd = deriv(xtraj,T0:Ts:Tf)'; qdd = qdd(:,nq+(1:nq));
% figure;plot(qddTrue(:,1),'-g'); hold on; plot(qdd(:,1),'-r')
% title(['q_1 True Acceleration vs. Derivative of Velocity']);
% xlabel('Sample Time')
% ylabel({'Acceleration Magnitude' '(m/s)/sample'})
% legend('True Acceleration','Derivative of Vel');

% v = AcrobotVisualizer(r);
% vtrue = AcrobotVisualizer(rtrue);
% vtrue.playback(xtraj);
% 
% % plot(xsamples(:,1)); hold on; plot(xsamplesfinal(:,1));
% plot(xsamples); hold on; plot(xsamplesfinal);
% % Testing to see if estimate is produced by local minima
% r = rtrue;

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