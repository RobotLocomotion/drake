r = Quadrotor();
tgen = WaypointTrajectoryLibraryGenerator(r);
tgen = tgen.setCyclicCoordinateIndexes([1,2,3,6]);

x0 = Point(r.getStateFrame);
x0.base_z = .5;
u0 = double(nominalThrust(r));

tgen = tgen.setInitialState(double(x0));
tgen = tgen.setInitialInput(u0);

xf = x0;
xf.base_x = 7;

tgen = tgen.setFinalState(double(xf));
tgen = tgen.setFinalInput(u0)

tgen = tgen.addWaypoint([1; 0; 0.5; 0]);
tgen = tgen.addWaypoint([2; 1; 0.5; 0]);
tgen = tgen.addWaypoint([3; -1; 0.5; 0]);
tgen = tgen.addWaypoint([4; 2; 0.5; 0]);
tgen = tgen.addWaypoint([5; -2; 0.5; 0]);
tgen = tgen.addWaypoint([6; 0; 0.5; 0])

tgen = tgen.setTrajectoryLength(11);
tgen.generateTrajectories;