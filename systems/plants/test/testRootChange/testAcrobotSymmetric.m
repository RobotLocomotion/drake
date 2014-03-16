% Create plant from urdf
p = RigidBodyManipulator('AcrobotSymmetric1.urdf');
N = p.num_q; 

figure(1);
subplot(1,2,1);
drawKinematicTree(p);

% Now, get new plant with root changed
pnew = changeRootLink(p, 'end_link', [0;0;0], [0;0;0], []);
% pnew = RigidBodyManipulator('AcrobotSymmetric2.urdf');

figure(1);
subplot(1,2,2);
drawKinematicTree(pnew);

kinsol = doKinematics(p,zeros(3,1));
xb = forwardKin(p,kinsol,findLinkInd(p,'base_link'),zeros(3,1));
xe = forwardKin(p,kinsol,findLinkInd(p,'end_link'),zeros(3,1));

kinsol_new = doKinematics(pnew,zeros(3,1));
xbnew = forwardKin(pnew,kinsol,findLinkInd(pnew,'base_link'),zeros(3,1));
xenew = forwardKin(pnew,kinsol,findLinkInd(pnew,'end_link'),zeros(3,1));
valuecheck(xb-xe,xbnew-xenew);


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
    x0 = Point(getStateFrame(p),randn(6,1));
    x0new = x0.inFrame(getStateFrame(pnew));
    
    % note: don't have to worry about the input frames in this case, only 
    % because there is just one input for this robot
    if (k<=100) 
      u0 = 0; 
    else
      u0 = randn(1);
    end
   
    xdot = Point(getStateFrame(pnew),pnew.dynamics(0,double(x0new),u0));
    xdot_true = Point(getStateFrame(p),p.dynamics(0,[pi;0;0;0;0;0]+double(x0),u0));
    
    valuecheck(xdot,xdot_true);
end

disp('Tests passed!');


