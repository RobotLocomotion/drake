function trigPolyTest

options.replace_output_w_new_state = true;

p1 = PlanarRigidBodyManipulator('../Acrobot.urdf');
tp1 = makeTrigPolySystem(p1,options);

oldpath=addpath('..');
p2 = AcrobotPlant();
path(oldpath);

tp2 = makeTrigPolySystem(p2,options);

[e1,f1]=getPolyDynamics(tp1);
[e2,f2]=getPolyDynamics(tp2);

if ~isequal(e1,e2) || ~isequal(f1,f2)  % is this supported by msspoly?
  error('dynamics are the same, but trig poly versions differ');
end

% if msspoly doesn't support equals, then i can test numerically... e.g.:
%for i=1:100
%  x0 = randn(4,1);
%  u0 = randn;
%
%  check that p1,p2,tp1,and tp2 all match
%end  