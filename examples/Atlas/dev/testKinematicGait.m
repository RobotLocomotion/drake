function testKinematicGait

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = PlanarRigidBodyManipulator('urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);

% periodic constraint
R_periodic = zeros(p.getNumStates,2*p.getNumStates);
R_periodic(2,2) = 1; %z
R_periodic(3,3) = 1; %pitch
R_periodic(4:6,8:10) = eye(3); %leg joints w/symmetry
R_periodic(8:10,4:6) = eye(3); %leg joints w/symmetry
R_periodic(7,7) = 1; % back joint
R_periodic(11:13,11:13) = eye(3); %x,z,pitch velocities
R_periodic(14:16,18:20) = eye(3); %leg joints w/symmetry
R_periodic(18:20,14:16) = eye(3); %leg joints w/symmetry
R_periodic(17,17) = 1; % back joint
R_periodic(2:end,p.getNumStates+2:end) = -eye(p.getNumStates-1);

tspan = [0,2];
q0 = [0;0;.2;-.4;.2;0;0;-1;2;0];
phi_tmp = p.contactConstraints(q0);
q0(2) = -phi_tmp(1);
x0 = [q0;.25;zeros(9,1)];
xf = -pinv(R_periodic(:,21:end))*(R_periodic(:,1:20)*x0);
xf(1) = .75;
nq = getNumPositions(p);
qf = xf(1:nq);
q_nom = PPTrajectory(foh(tspan,[q0,qf]));

l_foot = p.findLinkInd('l_foot');
r_foot = p.findLinkInd('r_foot');
world = p.findLinkInd('world');

ikoptions = IKoptions(p);
ikoptions = ikoptions.setDebug(false);
ikoptions = ikoptions.setMex(true);
ikoptions = ikoptions.setMajorIterationsLimit(1000);

xf = [0; 0.02; 0.1; 0.2];
zf = [0.0815; 0.0815+0.05; 0.0815+0.02; 0.0815];
pitch = [0; 0.1; 0.01; 0.0];
for i=1:4
  l_foot_con = WorldPositionConstraint(p,l_foot,[0;0;0],[xf(i);NaN;zf(i)],[xf(i);NaN;zf(i)],tspan);
  l_foot_pose_con = WorldEulerConstraint(p,l_foot,[NaN;pitch(i);NaN],[NaN;pitch(i);NaN],tspan);
  r_foot_con = WorldPositionConstraint(p,r_foot,[0;0;0],[0;NaN;0.0815],[0;NaN;0.0815],tspan);
  r_foot_pose_con = WorldEulerConstraint(p,r_foot,[NaN;0;NaN],[NaN;0;NaN],tspan);

  ikproblem = InverseKinematics(p,q_nom.eval(0),l_foot_con,l_foot_pose_con, ...
    r_foot_con,r_foot_pose_con);
  ikproblem = ikproblem.setQ(ikoptions.Q);
  ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
  ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);
  ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
  [qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q0);
  v = p.constructVisualizer();
  v.draw(0,qik);
  
  keyboard
end

