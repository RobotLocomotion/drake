function bodyKinTest()
checkGradients();
checkMex(createAtlas('rpy'));
end

function checkGradients()
options.floating = false;
r = RigidBodyManipulator(fullfile(getDrakePath(),'examples','KneedCompassGait','KneedCompassGait.urdf'),options);
m = 1;
pts = ones(3,m);
body_ind = 4;

q = rand*ones(r.getNumPositions(),1);

geval_options.grad_method = {'user','taylorvar'};
[x,J,dJ] = geval(@(q)bodyKinWrapper(q,r,body_ind,pts),q,geval_options);
[P,dP] = geval(@(q)bodyKinPWrapper(q,r,body_ind,pts),q,geval_options);
end

function [x,J,dJ] = bodyKinWrapper(q,r,body_ind,pts)
kinsol_options.use_mex = false;
if nargout > 2
  kinsol_options.compute_gradients = true;
end
kinsol = r.doKinematics(q, [], kinsol_options);

switch nargout
  case 1
    x = r.bodyKin(kinsol,body_ind,pts);
  case 2
    [x, ~, J] = r.bodyKin(kinsol,body_ind,pts);
  case 3
    [x, ~, J, ~, dJ] = r.bodyKin(kinsol,body_ind,pts);
end
end

function [P,dP] = bodyKinPWrapper(q,r,body_ind,pts)
kinsol_options.use_mex = false;
if nargout > 1
  kinsol_options.compute_gradients = true;
end
kinsol = r.doKinematics(q, [], kinsol_options);

switch nargout
  case 1
    [~, P] = r.bodyKin(kinsol,body_ind,pts);
  case 2
    [~, P, ~, dP] = r.bodyKin(kinsol,body_ind,pts);
end
end

function checkMex(robot)
nb = length(robot.body);
body_range = [1, nb];

n_tests = 5;
for i = 1 : n_tests
  body_ind = randi(body_range);
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  
  kinsol_options.use_mex = false;
  kinsol_options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], kinsol_options);
  [x, P, J, dP, dJ] = robot.bodyKin(kinsol,body_ind,points);
  
  kinsol_options.use_mex = true;
  kinsol_mex = robot.doKinematics(q, [], kinsol_options);
  [x_mex, P_mex, J_mex, dP_mex, dJ_mex] = robot.bodyKin(kinsol_mex,body_ind,points);
  valuecheck(x_mex, x);
  valuecheck(P_mex, P);
  valuecheck(J_mex, J);
  valuecheck(dP_mex, dP);
  valuecheck(dJ_mex, dJ);
end
end
