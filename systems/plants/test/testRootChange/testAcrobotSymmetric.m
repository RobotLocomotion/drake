% Create plant from urdf
p = RigidBodyManipulator('AcrobotSymmetric1.urdf');
N = p.num_q; 

% Construct visualizer
v = p.constructVisualizer;

% Now, get new plant with root changed
pnew = changeRootLink(p, 'end_link', [0;0;0], [0;0;0], []);
% pnew = RigidBodyManipulator('AcrobotSymmetric2.urdf');

% Get plant with root changed manually
% pnew_true = RigidBodyManipulator('AcrobotSymmetric2.urdf');

% First make sure origin is still fixed point
if (norm(pnew.dynamics(0,zeros(2*N,1),0)) > eps)
    error('Origin not a fixed point any more.');
end

% Now, compare dynamics at a bunch of points with no actuation
numTest = 100;
for k = 1:numTest
    x0 = randn(6,1);
   
    xdot = pnew.dynamics(0,x0,0);
    xdot_true = p.dynamics(0,[pi;0;0;0;0;0]+x0,0);
    
    if norm(xdot - xdot_true) > 1e-10
        error('Dynamics did not match');
    end
end

% Now, with actuation
numTest = 100;
for k = 1:numTest
    x0 = randn(6,1);
    u0 = randn(1,1);
   
    xdot = pnew.dynamics(0,x0,u0);
    xdot_true = p.dynamics(0,[pi;0;0;0;0;0]+x0,u0);
    
    if norm(xdot - xdot_true) > 1e-10
        error('Dynamics did not match');
    end
end

disp('Tests passed!');


