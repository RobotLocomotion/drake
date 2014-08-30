function quadrotorDynamics

w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning(w);

% URDF model and hand-written model from Penn
addpath(fullfile(pwd,'..'));
p1 = RigidBodyManipulator('../quadrotor.urdf',struct('floating',true));
p2 = QuadPlantPenn();

% Compare dynamics
for k = 1:100
    x0 = randn(12,1);
    u0 = randn(4,1).^2;
    
    xdot1 = p1.dynamics(0,x0,u0);
    xdot2 = p2.dynamics(0,x0,u0);
    
    valuecheck(xdot1,xdot2,1e-6);
    
end

end