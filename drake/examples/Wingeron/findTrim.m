function [ sol,exitflag ] = findTrim(X0)
%Finds a "trim" position for the MURI plane
%   Runs fmincon to find steady-state
%   values for the plane in forward flight.  The X-position state is
%   discarded, but all other states are minimized in order to find this
%   "fixed point" for the plane
%   Note: this seems to work.  One output is:
%   XYZ = [0 0 0]
%   RPY = [-0.0791 -0.2003 -0.0936]
%   LWing, RWing, Elev, Rud = [0.0000 -0.0057 0.2078 -0.0006]
%
%   Derivatives:
%   [14.4165 -0.0007 0]
%   [0 0 0]
%   [0 0 0 0]
%
%Torques[LWing   RWing   Elev   Rudder  Thrust]
%   U = [-0.0187 -0.0169 0.0020 -0.0095 573.0327]
%
%   Perhaps this should take the approach that findFixedPoint takes
%   in RigidBodyManipulator, such that the function to minimize always
%   returns zero, and instead the dynamics are included in the nonlinear
%   constraints to the problem.

options.floating = true;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
p = RigidBodyManipulator('Plane.URDF', options);
warning(w);
if nargin<1
    %              [X  Y  Z Rx  Ry   Rz WL WR EL Rd Vx   Vy Vz Vr Vp Vy Velev
    %These were an initial state tuned by hand
    %initialState = [0  0  0 0  -.15  0  0  0  .17 0 15.7 0  0  0  0  0  0 0 0 0]';
    %u0 = [-.0155 -.0155 .002 -.0005 380]';
    %A good initial state from running findTrim a few times:
    initialState= [0 0 0 -0.0791 -0.2003 -0.0936 0.0000 -0.0057 0.2078 -0.0006 14.4165 -0.0007 0 0 0 0 0 0 0 0]';
    u0 = [-0.0187 -0.0169 0.0020 -0.0095 573.0327]';
    %input to the minimization function needs to be x = [X;U]
    X0 = [initialState;u0];
end
options.MaxFunEvals = 10000;
options.Algorithm = 'active-set';
A = zeros(25);
b = zeros(25,1);
tol = 1E-2;
%    [X Y Z Rx  Ry  Rz WL WR EL Rd Vx Vy  Vz  VR  VP  VY  Velev   inputs
lb = [0 0 0 -.1 -1 -.1 -1 -1 -1 -1 10 -tol 0  0   0   0   0 0 0 0 -2 -2 -2 -2 0]';
ub = [0 0 0  .1  1  .1  1  1  1  1 20 tol  0  0   0   0   0 0 0 0 2  2  2  2  700]';
%                          function x0, A,b, Aeq,beq,lb,ub,nlcon,options
[sol,~,exitflag] = fmincon(@minfun, X0, A,b, [], [], lb, ub, [],options);

    function y = minfun(inX)
        x = inX(1:20);
        u = inX(21:25);
        xdot = p.dynamics(0,x,u);
%[X  Y  Z  Rx Ry Rz WL WR EL Rd Vx Vy Vz Vr Vp Vy Velev]
        y = sum(xdot(2:16).*xdot(2:16));
        y = y +.01*(sum(xdot(17:20).*xdot(17:20)));
    end
end

%visualizer bug
