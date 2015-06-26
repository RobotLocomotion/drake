r = Quadrotor();
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
tgen = tgen.setFinalInput(u0)

tgen = tgen.setTrajectoryLength(11);
[xtrajs, utrajs] = tgen.generateTrajectories;

for i = 1:numel(xtrajs)
  evalTraj = xtrajs{i}.eval(xtrajs{i}.getBreaks);
  plot(evalTraj(1,:), evalTraj(2,:))
  hold on;
end

disp('done')

