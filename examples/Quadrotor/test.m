r = Quadrotor();

if(~exist('trajectoryLibrary.mat'))
    %generate trajectory library
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
    disp('building trajectory library...')
    [xtrajs, utrajs] = tgen.generateTrajectories;
    save 'trajectoryLibrary.mat' xtrajs utrajs
else
    load('trajectoryLibrary.mat');
    for i = 1:numel(xtrajs)    
        xtrajs{i} = xtrajs{i}.setOutputFrame(r.getStateFrame);
        utrajs{i} = utrajs{i}.setOutputFrame(r.getInputFrame);
    end
end

%plot trajectory library
for i = 1:numel(xtrajs)
  evalTraj = xtrajs{i}.eval(xtrajs{i}.getBreaks);
  plot(evalTraj(1,:), evalTraj(2,:))
  hold on;
end

xtraj = xtrajs{1};
utraj = utrajs{1};

disp('building tvlqr controller...')
%compute stabilizing controller for the first trajectory in library
[tv,Vtv] = tvlqr(r, xtraj, utraj, eye(12), eye(4), eye(12), struct());

%assemble closed loop system
sys_closed_loop = feedback(r, tv);

%form polynomial approximations to the system dynamics for both closed loop
%and open loop systems.  A 3rd order Taylor series expansion is used here.

disp('taylor approximating system dynamics...')
sys_poly_closed_loop = taylorApprox(sys_closed_loop, xtraj, [], 3);
sys_poly_open_loop = taylorApprox(r, xtraj, utraj, 3);

ts = Vtv.S.getBreaks(); tsend = ts(end);
ts = linspace(ts(1),ts(end),12);
ts = ts(1:end-1);


G0 = eye(12);
disp('computing funnels...')
options = struct();
options.saturations = false;
options.rho0 = 1;
options.degL1 = 2;
options.max_iterations = 10;
[V,rho,Phi]=sampledFiniteTimeReach(sys_poly_closed_loop,sys_poly_open_loop,Vtv/10,G0,tv,ts,xtraj,utraj,options);

disp('done');

