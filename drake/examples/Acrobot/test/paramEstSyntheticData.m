function paramEstSyntheticData
mode = 2;
options.print_result = 0;

noisestd = sqrt([10, 10, 20, 20]);

r = AcrobotPlant;
rtest = AcrobotPlant;
% v = AcrobotVisualizer(r);
% vtest = AcrobotVisualizer(rtest);

% Perturb the original parameter estimates with normally dist. random error
% with variance 
errstd = 1/10;
paramerr = randn(1,10)*errstd;
paramerr(paramerr>1) = 1;
paramerr(paramerr<-1) = -1;
% disp(paramerr)

rtest.l1 = rtest.l1 + rtest.l1*paramerr(1); 
rtest.l2 = rtest.l2 + rtest.l2*paramerr(2); 
rtest.m1 = rtest.m1 + rtest.m1*paramerr(3); 
rtest.m2 = rtest.m2 + rtest.m2*paramerr(4);  
rtest.b1 = rtest.b1 + rtest.b1*paramerr(5);
rtest.b2 = rtest.b2 + rtest.b2*paramerr(6);
rtest.lc1 = rtest.lc1 + rtest.lc1*paramerr(7); 
rtest.lc2 = rtest.lc2 + rtest.lc2*paramerr(8); 
rtest.Ic1 = rtest.Ic1 + rtest.Ic1*paramerr(9);  
rtest.Ic2 = rtest.Ic2 + rtest.Ic2*paramerr(10);


%% test on swingup up data
[utraj,xtraj] = swingUpTrajectory(r);

Ts = .01; breaks=getBreaks(utraj); T0 = breaks(1); Tf = breaks(end);

% Try parameter estimation without any noise
if mode ==1
    data = iddata(eval(xtraj,T0:Ts:Tf)',eval(utraj,T0:Ts:Tf)',Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',r.getOutputFrame.getCoordinateNames());
    estimated_parameters = parameterEstimation(rtest,data,options);
end

% Try parameter estimation with measurement noise
if mode == 2
    xsamples = eval(xtraj,T0:Ts:Tf)';
    xnoisysamples = xsamples + randn(size(xsamples))*diag(noisestd);
%     disp(xnoisysamples-xsamples);
    usamples = eval(utraj,T0:Ts:Tf)';
    data = iddata(xnoisysamples,usamples,Ts,'InputName',r.getInputFrame.getCoordinateNames(),'OutputName',r.getOutputFrame.getCoordinateNames());
    estimated_parameters = parameterEstimation(rtest,data,options);
end


% Try parameter estimation with noise and known delay



% print out results
coords = getCoordinateNames(getParamFrame(r));
p_orig = double(getParams(r));
fprintf('\nParameter estimation results:\n\n');
fprintf('  Param  \tOriginal\tEstimated\n');
fprintf('  -----  \t--------\t---------\n');
for i=1:length(coords)
  fprintf('%7s  \t%8.2f\t%8.2f\n',coords{i},p_orig(i),estimated_parameters(i));
end
% disp(estimated_parameters);
