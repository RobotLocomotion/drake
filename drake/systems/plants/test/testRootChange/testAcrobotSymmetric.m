% Create plant from urdf
p = RigidBodyManipulator('AcrobotSymmetric1.urdf');
N = p.num_positions; 


% Now, get new plant with root changed
pnew = changeRootLink(p, 'end_link', [0;0;0], [0;0;0], []);
% pnew = RigidBodyManipulator('AcrobotSymmetric2.urdf');

%figure(1);
%subplot(1,2,1);
%drawKinematicTree(p);
%subplot(1,2,2);
%drawKinematicTree(pnew);

%v = constructVisualizer(pnew);
%v.inspector
%keyboard;

for k=1:100
  if k<=1, 
    q0 = zeros(3,1);
  else
    q0 = randn(3,1);
  end
  
  link_ind = randi(4);
  b = getBody(p,link_ind);
  kinsol = doKinematics(p,q0);
  pt = forwardKin(p,kinsol,link_ind,b.com);

  b = getBody(pnew,link_ind);
  kinsol_new = doKinematics(pnew,[pi;0;0]-q0);%,false,false);
  pt_new = forwardKin(pnew,kinsol_new,link_ind,b.com);
  
  valuecheck(pt_new,pt);
end

% Get plant with root changed manually
%pnew_true = RigidBodyManipulator('AcrobotSymmetric2.urdf');

% First make sure origin is still fixed point
if (norm(pnew.dynamics(0,zeros(2*N,1),0)) > eps)
    error('Origin not a fixed point any more.');
end

% Compare dynamics at a bunch of points with no actuation, then with
% actuation
numTest = 200;
for k = 1:numTest
    % note: we're actually intentionally abusing the frames here, because
    % the system is symmetric
    x0 = randn(6,1);
    
    if (k<=100) 
      u0 = 0; 
    else
      u0 = randn(1);
    end
   
    xdot = pnew.dynamics(0,x0,u0);
    xdot_true = -p.dynamics(0,[pi;0;0;0;0;0]-x0,-u0);
    
    valuecheck(xdot,xdot_true);
end

disp('Tests passed!');


