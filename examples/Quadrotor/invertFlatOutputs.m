function [xtraj,utraj] = invertFlatOutputs(plant,ytraj,options)
% NOTEST
% @param plant a quadrotor plant (either Quadrotor or QuadPlantPenn)
% @param ytraj a PPTrajectory in the DifferentiallyFlatOutputFrame
% @option zero_acceleration_tol threshold on the magnitude of the 
%                        acceleration vector below which the quad is
%                        considered stationary. @default 1e-8
% @option traj_freq frequency of the generated xtraj and utraj in Hz.
%                        @default 200
%
% implementation based on 
% Minimum Snap Trajectory Generation and Control for Quadrotors
% by Daniel Mellinger and Vijay Kumar
% ICRA 2011

typecheck(plant,{'Quadrotor','QuadPlantPenn'});
typecheck(ytraj,'PPTrajectory');
assert(ytraj.getOutputFrame==DifferentiallyFlatOutputFrame);

if nargin<3, options=struct(); end
if ~isfield(options,'zero_acceleration_tol'), options.zero_acceleration_tol = 1e-8; end
if ~isfield(options,'traj_freq'), options.traj_freq = 200; end

ydtraj = fnder(ytraj);
yddtraj = fnder(ydtraj);
ydddtraj = fnder(yddtraj);
yddddtraj = fnder(ydddtraj);
t0 = ytraj.tspan(1);
tf = ytraj.tspan(2);
tt = linspace(t0,tf,ceil((tf-t0)*options.traj_freq));
xx = zeros(12,numel(tt));
uu = zeros(4,numel(tt));
for i=1:numel(tt)
    [xx(:,i),uu(:,i)] = extractStateAndInput(plant,tt(i),ytraj,ydtraj,yddtraj,ydddtraj,yddddtraj,options);
end
xtraj = PPTrajectory(spline(tt,xx));
xtraj = setOutputFrame(xtraj,getStateFrame(plant));
utraj = PPTrajectory(spline(tt,uu));
utraj = setOutputFrame(utraj,getInputFrame(plant));
end

function [x,u] = extractStateAndInput(plant,t,ytraj,ydtraj,yddtraj,ydddtraj,yddddtraj,options)
    g = plant.getGravity;
    zW = g/norm(g);
    robotnum = plant.getBody(plant.findLinkId('base_link')).robotnum;
    m = plant.getMass(robotnum);
    I = plant.getInertia;
    y = ytraj.eval(t);
    yd = ydtraj.eval(t);
    ydd = yddtraj.eval(t);
    yddd = ydddtraj.eval(t);
    ydddd = yddddtraj.eval(t);
    a = ydd(1:3)-g;
    ad = yddd(1:3);
    add = ydddd(1:3);
    u1 = m*norm(a);
    u1d = m*(ad'*a)/norm(a);
    u1dd = m*((add'*a+ad'*ad)*norm(a)-(ad'*a)*(ad'*a)/norm(a))/norm(a)^2;

    if norm(a)<options.zero_acceleration_tol
        zB = [0;0;1];
        zBd = [0;0;0];
    else
        zB = a/norm(a);
        zBd = (ad*norm(a)-a*(ad'*a)/norm(a))/norm(a)^2;
    end
    xC = [cos(y(4));sin(y(4));0];
    xCd = [-sin(y(4))*yd(4);cos(y(4))*yd(4);0];
    yB = cross(zB,xC);
    yBd = cross(zBd,xC)+cross(zB,xCd);
    yBd = (yBd*norm(yB)-yB*((yBd'*yB)/norm(yB)))/norm(yB)^2;
    yB = yB/norm(yB);
    xB = cross(yB,zB);
    xBd = cross(yBd,zB)+cross(yB,zBd);

    R = [xB,yB,zB];
    x = [y(1:3); rotmat2rpy(R); yd(1:3); zeros(3,1)];

    h_omega = (1/u1)*(m*ad-u1d*zB);
    omegaBW = [ -h_omega'*yB; h_omega'*xB; yd(4)*zW'*zB ];

    x(10:12) = angularvel2rpydot(x(4:6), omegaBW);

    h_omegad = (m*add*u1-m*ad*u1d-(u1dd*zB+u1d*zBd)*u1+u1d*zB*u1d)/u1^2;
    omegadotBW = [ -h_omegad'*yB-h_omega'*yBd; h_omegad'*xB+h_omega'*xBd; ydd(4)*zW'*zB ];
    
    % Euler's equation to find the corresponding inputs
    mellinger_u = [u1; I*omegadotBW + cross(omegaBW,(I*omegaBW))];

    % note: u vector = kf*omega.^2 from mellinger
    kF = zeros(1,4);
    kM = zeros(1,4);
    L = zeros(1,4);
    for i=1:4
        prop = plant.force{i};
        kF(i) = abs(prop.scale_factor_thrust);
        kM(i) = abs(prop.scale_factor_moment);
        frame = getFrame(plant,prop.kinframe);
        com = getCOM(plant,zeros(getNumPositions(plant),1));
        L(i) = norm(frame.T(1:3,4)-com);
    end
    omega_squared = [kF; 0, kF(2)*L(2), 0, -kF(4)*L(4); -kF(1)*L(1), 0, kF(3)*L(3), 0; kM(1), -kM(2), kM(3), -kM(4)]\mellinger_u;
    u = omega_squared;
end
