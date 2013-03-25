function traj = SimpleWave

fr = HuboJointCommand;
traj = ConstantTrajectory(Point(fr));
t = 0:.1:5;
traj(8) = PPTrajectory(spline(t,.05*sin(2*pi*t/5)));  % left elbow
traj = setOutputFrame(traj,fr);


