function environmentTest

% loads some drc objects, just for fun

r = TimeSteppingRigidBodyManipulator('',.001);
r = addRobotFromURDF(r,'../urdf/atlas_minimal_contact.urdf',[0;0;0],[],struct('floating',true));
%r = addRobotFromURDF(r,'/Users/russt/drc/software/models/mit_gazebo_models/modified_golf_cart/model3.sdf');
r = addRobotFromURDF(r,'steering.urdf',[.6;0;0]);
v = r.constructVisualizer();

lhand = findLink(r,'l_hand',0,true);
rhand = findLink(r,'r_hand',0,true);
rfoot = findLink(r,'r_foot');
lfoot = findLink(r,'l_foot');

wheel = findLink(r,'steering_wheel',0,true);

cost = Point(r.getStateFrame,1);
cost.pelvis_x = 100;
cost.pelvis_y = 100;
cost.pelvis_z = 100;
cost.pelvis_roll = 1000;
cost.pelvis_pitch = 1000;
cost.pelvis_yaw = 0;
cost.back_mby = 100;
cost.back_ubx = 100;
cost = double(cost);
options = struct();
options.Q = diag(cost(1:r.getNumDOF));
load('../data/atlas_fp3.mat');
q0 = [xstar(1:34);0];
options.q_nom = q0;

v.draw(0,[q0;0*q0]);

kinsol = doKinematics(r,q0);
x = forwardKin(r,kinsol,wheel,[[0.1;-.2;-0.05],[0.1;.2;-0.05]]);

rfoot0 = forwardKin(r,kinsol,rfoot,[0;0;0],true);
lfoot0 = forwardKin(r,kinsol,lfoot,[0;0;0],true);


q = inverseKin(r,q0,rhand,x(:,1),lhand,x(:,2),rfoot,rfoot0,lfoot,lfoot0);
v.draw(0,[q;0*q]);
