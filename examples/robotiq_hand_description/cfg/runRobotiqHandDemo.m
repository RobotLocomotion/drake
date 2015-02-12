function runRobotiqHandDemo
% Simulate the Robotiq hand grasping a cylinder

% Load the model with a fixed base
options.floating = true;
options.dt = 0.001;
options.viewer = 'RigidBodyWRLVisualizer';

r = TimeSteppingRigidBodyManipulator([],options.dt);
%r = r.addRobotFromURDF('s-model_articulated_fourbar.urdf',[0;0;0],[0;0;0]);
r = r.addRobotFromURDF('s-model_articulated_fourbar_remove_package.urdf',[0;0;0],[0;0;0]);
r = r.addRobotFromURDF('cylinder.urdf',[0;.1;0],[0;0;0]);
r = compile(r);

% Initialize the viewer
v = r.constructVisualizer;

% Compute a feasible set of initial conditions for the simulation 
x0 = Point(r.getStateFrame);

x0.finger_1_fourbar_0_joint_0 = -2.10;
x0.finger_1_fourbar_0_joint_1 = 1.881;
x0.finger_1_fourbar_0_joint_2 = -1.997;
x0.finger_1_fourbar_3_joint_1 = 1.613;

x0.finger_2_fourbar_0_joint_0 = -2.10;
x0.finger_2_fourbar_0_joint_1 = 1.881;
x0.finger_2_fourbar_0_joint_2 = -1.997;
x0.finger_2_fourbar_3_joint_1 = 1.613;

x0.finger_middle_fourbar_0_joint_0 = -2.10;
x0.finger_middle_fourbar_0_joint_1 = 1.881;
x0.finger_middle_fourbar_0_joint_2 = -1.997;
x0.finger_middle_fourbar_3_joint_1 = 1.613;

%x0.palm_finger_1_joint = 0;
%x0.palm_finger_2_joint = 0;

x0 = resolveConstraints(r,x0);

v.draw(0,double(x0));

%k_d = 0.5*eye(5);
%k_p = 0.5*eye(5);
k_d = 0.5*eye(3);
k_p = 0.5*eye(3);

if (1) % Run simulation, then play it back at realtime speed
  tic;
  sys=r.pdcontrol(k_p,k_d,r.getActuatedJoints);
  %qdtraj = ConstantTrajectory(Point(sys.getInputFrame,[-0.255;-0.255;-0.255;0;0]));
  qdtraj = ConstantTrajectory(Point(sys.getInputFrame,[-0.255;-0.255;-0.255]));
  sys = cascade(qdtraj,sys);
  xtraj = simulate(sys,[0 2],x0);
  toc;
  playback(v,xtraj,struct('slider',true));
  %playbackMovie(v,xtraj,'ballDemo.mp4');
else % View the simulation as it is being computed
  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  
  sys = cascade(r,v);
  simulate(sys,[0 2],x0);
end
end