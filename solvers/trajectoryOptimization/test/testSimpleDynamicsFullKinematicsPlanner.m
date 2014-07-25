function testSimpleDynamicsFullKinematicsPlanner
warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_minimal_contact.urdf'];
options.floating = true;
robot = RigidBodyManipulator(urdf,options);
nomdata = load([getDrakePath,'/examples/Atlas/data/atlas_fp.mat']);
nq = robot.getNumPositions();
qstar = nomdata.xstar(1:nq);
nv = robot.getNumDOF();
vstar = zeros(nv,1);

l_foot = robot.findLinkInd('l_foot');
r_foot = robot.findLinkInd('r_foot');
l_foot_shapes_heel = robot.getBody(l_foot).getContactShapes('heel');
l_foot_shapes_toe = robot.getBody(l_foot).getContactShapes('toe');
r_foot_shapes_heel = robot.getBody(r_foot).getContactShapes('heel');
r_foot_shapes_toe = robot.getBody(r_foot).getContactShapes('toe');
l_foot_toe = [];
l_foot_heel = [];
r_foot_toe = [];
r_foot_heel = [];
for i = 1:length(l_foot_shapes_heel)
  l_foot_heel = [l_foot_heel l_foot_shapes_heel{i}.getPoints];
end
for i = 1:length(l_foot_shapes_toe)
  l_foot_toe = [l_foot_toe l_foot_shapes_toe{i}.getPoints];
end
for i = 1:length(r_foot_shapes_heel)
  r_foot_heel = [r_foot_heel r_foot_shapes_heel{i}.getPoints];
end
for i = 1:length(r_foot_shapes_toe)
  r_foot_toe = [r_foot_toe r_foot_shapes_toe{i}.getPoints];
end
l_foot_bottom = [l_foot_toe l_foot_heel];
r_foot_bottom = [r_foot_toe r_foot_heel];

l_foot_contact_cnstr = cell(2,1);
l_foot_contact_cnstr{1} = FrictionConeWrenchConstraint(robot,l_foot,l_foot_heel,1,[0;0;1],[0,0.7]);
l_foot_contact_cnstr{2} = FrictionConeWrenchConstraint(robot,l_foot,l_foot_toe,1,[0;0;1],[0.3,1]);

r_foot_contact_cnstr = cell(2,1);
FC_theta = linspace(0,2*pi,5);
FC_edge = [sin(FC_theta);cos(FC_theta);ones(1,5)];
r_foot_contact_cnstr{1} = LinearFrictionConeWrenchConstraint(robot,r_foot,r_foot_heel,FC_edge,[0.6,1.3]);
r_foot_contact_cnstr{2} = LinearFrictionConeWrenchConstraint(robot,r_foot,r_foot_toe,FC_edge,[0.9,1.6]);

q_nom_traj = ConstantTrajectory(qstar);
tf_range = [1.6 2.5];
sdfkp = ComDynamicsFullKinematicsPlanner(robot,linspace(0,1.6,17),tf_range,q_nom_traj,true,nomdata.xstar,l_foot_contact_cnstr{:},r_foot_contact_cnstr{:});
end