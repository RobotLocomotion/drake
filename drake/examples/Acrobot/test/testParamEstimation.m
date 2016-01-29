function testParamEstimation
% Unit test to ensure functionality of parameter estimate
% Perturbs the initial parameters guess by a random gaussian percentage error
% Does not add measurement noise

tmp = addpathTemporary(fullfile(pwd,'..'));

% Setting a fixed seed to avoid stochastic failures
rng(3);

testThreshold = 5e-2;

parameterEstimationOptions.method = 'nonlinprog'; % nonlinear programming
parameterEstimationOptions.C = eye(4); % The symmetric positive definite cost matrix for simerr
paramstd = 0.01; % Standard deviation of the parameter value percent error

numTests = 1;
models = {'energetic','dynamic','simerr'};
mlen = length(models);

rtrue = AcrobotPlant;
p_true = double(rtrue.getParams);

%% Test on swingup up data
[utraj,xtraj] = swingUpTrajectory(rtrue);
Ts = .002; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);
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

    %% Generate second derivative
    % Differentiating to get the second derivative of the state variables
    dtsamples = diff(tsamples)';
    qdd = diff(xsamples(:,nq+(1:nq)),1,1)./repmat(dtsamples,1,nq);
    xsamples = [xsamples(1:length(qdd),:), qdd];
    data = iddata(xsamples,usamples(1:length(qdd),:),Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',outputFrameNames);

    %% Compute results from each model    
    for i=1:mlen
        parameterEstimationOptions.model = models{i};
        tic;
        [p_est{i}(:,j),simerror(i,j),~,simflag(i,j)] = parameterEstimation(r,data,parameterEstimationOptions);
        simtime(i,j) = toc;
%         fprintf('n: %i \tModel: %10s \tTime: %f \tSim Error: %f\n',j,models{i},simtime(i,j),simerror(i,j));
%         %% Print out results
%         fprintf('\nParameter estimation results for %s:\n\n',models{i});
%         fprintf('  Param  \tTrue    \t Initial\tEstimated\n');
%         fprintf('  -----  \t--------\t--------\t---------\n');
%         for k=1:length(paramNames)
%           fprintf('%7s  \t%8.2f\t%8.2f\t%8.2f\n',paramNames{k},p_true(k),p_init(k),p_est{i}(k,j));
%         end
        
        if any(((p_true-p_est{i}(:,j))./p_true).^2 > testThreshold)
            disp(((p_true-p_est{i}(:,j))./p_true).^2);
            warning(char(strcat(models(i),' did not pass the test')));
        end
    end
end
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