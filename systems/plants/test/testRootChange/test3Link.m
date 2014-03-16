% Create plant from urdf
p = RigidBodyManipulator('3LinkThing.urdf');
N = p.num_q; 

% Construct visualizer
v = p.constructVisualizer;
x0 = [0;pi/3;-pi/3;0;0;0];
v.draw(0,x0);

% Now, get new plants with roots changed
pnew1 = changeRootLink(p, 'link2', [0;0;1], [0;0;0], []);
pnew2 = changeRootLink(p, 'link3', [0;0;1], [0;0;0], []);

% Now, compare dynamics at a bunch of points
numTest = 100;
for k = 1:numTest
    x0 = randn(6,1);
    u0 = randn(1,1);
    
    xdot_orig = p.dynamics(0,x0,u0*ones(3,1));
    xdotnew1 = pnew1.dynamics(0,x0,u0*ones(3,1));
    xdotnew2 = pnew2.dynamics(0,x0,u0*ones(3,1));
    
    
    if norm(xdot_orig - xdotnew1) > 1e-10
        error('Dynamics did not match');
    end
    
    if norm(xdot_orig - xdotnew2) > 1e-10
        error('Dynamics did not match');
    end
    
end

disp('Tests passed!');




