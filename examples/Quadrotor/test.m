r = Quadrotor();

if(~exist('trajectoryLibrary.mat'))
    %generate trajectory library
    disp('building trajectory library...')
    tgen = WaypointTrajectoryLibraryGenerator(r);
    tgen = tgen.setCyclicCoordinateIndexes([1,2,3,6]);

    x0 = Point(r.getStateFrame);
    x0.base_z = .5;
    u0 = double(nominalThrust(r));

    tgen = tgen.setInitialState(double(x0));
    tgen = tgen.setInitialInput(u0);

    relativePositions = [cos(pi/6*(-2:2));sin(pi/6*(-2:2))];
    start = [1;0];

    tgen = tgen.addWaypoint([start;0.5;0]);
    
    for i = 1:size(relativePositions,2)
        start = start + relativePositions(:, i);
        tgen = tgen.addWaypoint([start;0.5;0]);
    end

    xf = x0;
    xf.base_x = start(1) + 1;

    tgen = tgen.setFinalState(double(xf));
    tgen = tgen.setFinalInput(u0);

    tgen = tgen.setTrajectoryLength(11);
    trajLib = tgen.generateTrajectories();
    

    save 'trajectoryLibrary.mat' trajLib
else
    disp('loading trajectoryLibrary.mat...');
    load('trajectoryLibrary.mat');
    trajLib = trajLib.setupFrames(r);
end

disp('stabilizing trajectories...');
Q = diag([10*ones(6,1);ones(6,1)]);
R = 0.1*eye(4);
Qf = diag([100*ones(6,1);ones(6,1)]);    
trajLib = trajLib.stabilizeTrajectories(r, Q, R, Qf);

V_funnel = {};
rho_funnel = {};
Phi_funnel = {};
for i = 1:numel(trajLib.trajectories)

  xtraj = trajLib.trajectories{i}.xtraj;
  utraj = trajLib.trajectories{i}.utraj;
  Vtv = trajLib.trajectories{i}.Vtv;
  tv = trajLib.trajectories{i}.tv;
  
disp('computing funnels...')

V = Vtv*95;

%assemble closed loop system
sys_closed_loop = feedback(r, tv);

%form polynomial approximations to the system dynamics for both closed loop
%and open loop systems.  A 3rd order Taylor series expansion is used here.
disp('taylor approximating system dynamics...')

sys_poly_closed_loop = taylorApprox(sys_closed_loop, xtraj, [], 3);
sys_poly_open_loop = taylorApprox(r, xtraj, utraj, 3);

ts = Vtv.S.getBreaks(); tsend = ts(end);
ts = linspace(ts(1),ts(end),12);
%ts = xtraj.getBreaks();
ts = ts(1:end-1);

options = struct();
options.saturations = false;
options.rho0 = 1;
options.degL1 = 2;
options.max_iterations = 1;
G0 = V.S.eval(0) / options.rho0 * 1.01;
options.backoff_percent = 5;

[V,rho,Phi]=sampledFiniteTimeReach_B0(sys_poly_closed_loop,sys_poly_open_loop,V,G0,0,tv,ts,xtraj,utraj,options);

V_funnel{i} = V;
rho_funnel{i} = rho;
Phi_funnel{i} = Phi;
end

for i = 1:numel(xtrajs)

Vxframe = V_funnel{i}.inFrame(r.getStateFrame());
options.plotdims = [1 2];
options.x0 = xtraj;
options.ts = ts;
options.inclusion = 'projection';
plotFunnel(Vxframe,options);
hold on;
fnplt(xtrajs{i},[1 2]); 

end


disp('done');
