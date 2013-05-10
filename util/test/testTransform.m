function testTransform

% unit test to make sure java Transform class stays in sync with the matlab
% equivalents

for i=1:50
  rpy = randn(3,1);
  valuecheck(rpy2quat(rpy),drake.util.Transform.rpy2quat(rpy));
end

for i=1:50
  quat = randn(4,1);  quat=quat/norm(quat);
  valuecheck(quat2rpy(quat),drake.util.Transform.quat2rpy(quat));
end

% special case that caught an error once before
rpy = [0;-pi/2;pi];
valuecheck(drake.util.Transform.quat2rpy(drake.util.Transform.rpy2quat(rpy)),rpy);

for i=1:50
  rpy = randn(3,1); rpydot = randn(3,1);
  valuecheck(rpydot2angularvel(rpy,rpydot),drake.util.Transform.rpydot2angularvel(rpy,rpydot));
end

for i=1:50
  rpy = randn(3,1); omega = randn(3,1);
  valuecheck(angularvel2rpydot(rpy,omega),drake.util.Transform.angularvel2rpydot(rpy,omega));
end
