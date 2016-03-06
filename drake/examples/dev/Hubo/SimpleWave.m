function traj = SimpleWave

fr = HuboJointCommand;
load hubo_traj.mat;
%traj=atraj;
%traj = ConstantTrajectory(traj);
%t = 0:.1:5;
%traj(5) = PPTrajectory(spline(t,.2*sin(2*pi*t/5)));  % left elbow
%traj(6) = PPTrajectory(spline(t,.2*sin(2*pi*t/5)));  % left elbow
%traj(10) = PPTrajectory(spline(t,.2*sin(2*pi*t/5)));  % left elbow
%traj(11) = PPTrajectory(spline(t,.2*sin(2*pi*t/5)));  % left elbow
traj = setOutputFrame(traj,fr);


