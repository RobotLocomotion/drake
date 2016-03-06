function testMotionSubspaceDotTimesV()

compareToUsingMotionSubspaceGradient();
testGradients();

end

function compareToUsingMotionSubspaceGradient()
for i = 1 : 100
  body.floating = 0;
  body.joint_axis = randn(3, 1);
  q = randn;
  v = randn;
  body.pitch = 0;
  compareToUsingMotionSubspaceGradientForBody(body, q, v);
  body.pitch = inf;
  compareToUsingMotionSubspaceGradientForBody(body, q, v);
  body.pitch = randn;
  compareToUsingMotionSubspaceGradientForBody(body, q, v);
  
  body.floating = 1;
  q = [uniformlyRandomRPY(); randn(3, 1)];
  v = randn(6, 1);
  compareToUsingMotionSubspaceGradientForBody(body, q, v);
  
  body.floating = 2;
  q = [uniformlyRandomQuat(); randn(3, 1)];
  v = randn(6, 1);
  compareToUsingMotionSubspaceGradientForBody(body, q, v);
end

end

function compareToUsingMotionSubspaceGradientForBody(body, q, v)
qd = jointV2qdot(body, q) * v;
[S, dSdq] = motionSubspace(body, q);
Sdot = reshape(dSdq * qd, size(S));
Sdot_times_v_check = Sdot * v;
Sdot_times_v = motionSubspaceDotTimesV(body, q, v);
valuecheck(Sdot_times_v_check, Sdot_times_v, 1e-12);
end

function testGradients()
option.grad_method = {'user', 'taylorvar'};

for i = 1 : 10
  body.floating = 0;
  body.joint_axis = randn(3, 1);
  q = randn;
  v = randn;
  
  body.pitch = 0;
  [~, ~] = geval(1, @(q) gevalFunQ(body, q, v), q, option);
  [~, ~] = geval(1, @(v) gevalFunV(body, q, v), v, option);
  body.pitch = inf;
  [~, ~] = geval(1, @(q) gevalFunQ(body, q, v), q, option);
  [~, ~] = geval(1, @(v) gevalFunV(body, q, v), v, option);
  body.pitch = randn;
  [~, ~] = geval(1, @(q) gevalFunQ(body, q, v), q, option);
  [~, ~] = geval(1, @(v) gevalFunV(body, q, v), v, option);
  
  body.floating = 1;
  q = [uniformlyRandomRPY(); randn(3, 1)];
  v = randn(6, 1);
  [~, ~] = geval(1, @(q) gevalFunQ(body, q, v), q, option);
  [~, ~] = geval(1, @(v) gevalFunV(body, q, v), v, option);
  
  body.floating = 2;
  q = [uniformlyRandomQuat(); randn(3, 1)];
  v = randn(6, 1);
  [~, ~] = geval(1, @(q) gevalFunQ(body, q, v), q, option);
  [~, ~] = geval(1, @(v) gevalFunV(body, q, v), v, option);
end
end

function [SdotV, dSdotVdq] = gevalFunQ(body, q, v)
[SdotV, dSdotVdq] = motionSubspaceDotTimesV(body, q, v);
end

function [SdotV, dSdotVdv] = gevalFunV(body, q, v)
[SdotV, ~, dSdotVdv] = motionSubspaceDotTimesV(body, q, v);
end
