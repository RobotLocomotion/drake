function testKinematicStepTraj

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
l = 0.4;
h = 0.11;
boxes = [0.25+l, 0.0, 2*l, 1, h;
         0.25+l+l/2, 0.0, l, 1, 2*h];
options.terrain = RigidBodyStepTerrain(boxes);
% options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
p = PlanarRigidBodyManipulator('../urdf/atlas_simple_spring_ankle_planar_contact.urdf',options);

T = 4.0;
tspan = [0,T];
q0 = [0;0;.2;-.4;.2;0;0;-1;2;0];
phi_tmp = p.contactConstraints(q0);
q0(2) = -phi_tmp(1);
nq = getNumPositions(p);
q_nom = q0;

l_foot = p.findLinkId('l_foot');
r_foot = p.findLinkId('r_foot');
pelvis = p.findLinkId('pelvis');

ikoptions = IKoptions(p);
ikoptions = ikoptions.setDebug(false);
ikoptions = ikoptions.setMex(true);
ikoptions = ikoptions.setMajorIterationsLimit(1000);

n_knots = 9;

% compute desired left foot positions 
fh = 0.0815;
ph = 0.85;

lfoot_knots = zeros(3,n_knots);
lfoot_knots(1,:) = [0, 0, l, ...
                    l, l, l, ...
                    l, 3*l, 3*l];
lfoot_knots(3,:) = [fh, fh+h+0.05, fh+h, ...
                    fh+h, fh+h, fh+h, ...
                    fh+2*h+0.05, fh+2*h+0.05, fh];
                  
rfoot_knots = zeros(3,n_knots);
rfoot_knots(1,:) = [0, 0, 0, ...
                    0, l, 2*l,...
                    2*l, 2*l, 2*l];
rfoot_knots(3,:) = [fh, fh, fh, ...
                    fh+h+0.05, fh+2*h+0.05, fh+2*h, ...
                    fh+2*h,  fh+2*h,  fh+2*h];

pelvis_knots = zeros(3,n_knots);
pelvis_knots(1,:) = [0, 0, l/2, ...
                     l, l, l+l/2, ...
                     2*l, 2*l, 2*l+l/2];
pelvis_knots(3,:) = [ph, ph, ph, ...
                     h+ph, h+ph, h+ph, ...
                     2*h+ph, 2*h+ph, ph];

                   
ts = linspace(0,T,n_knots);
lfoot_traj = PPTrajectory(foh(ts,lfoot_knots));                   
rfoot_traj = PPTrajectory(foh(ts,rfoot_knots));                   
pelvis_traj = PPTrajectory(foh(ts,pelvis_knots));                   

qs = zeros(nq,n_knots);

ts = linspace(0,T,100);
for i=1:length(ts)
  lfoot_des = lfoot_traj.eval(ts(i));
  rfoot_des = rfoot_traj.eval(ts(i));
  pelvis_des = pelvis_traj.eval(ts(i));
  l_foot_con = WorldPositionConstraint(p,l_foot,[0;0;0],[lfoot_des(1);NaN;lfoot_des(3)],[lfoot_des(1);NaN;lfoot_des(3)],tspan);
  l_foot_pose_con = WorldEulerConstraint(p,l_foot,[NaN;0;NaN],[NaN;0;NaN],tspan);
  r_foot_con = WorldPositionConstraint(p,r_foot,[0;0;0],[rfoot_des(1);NaN;rfoot_des(3)],[rfoot_des(1);NaN;rfoot_des(3)],tspan);
  r_foot_pose_con = WorldEulerConstraint(p,r_foot,[NaN;0;NaN],[NaN;0;NaN],tspan);
  pelvis_con = WorldPositionConstraint(p,pelvis,[0;0;0],[pelvis_des(1);NaN;pelvis_des(3)],[pelvis_des(1);NaN;pelvis_des(3)],tspan);
  pelvis_pose_con = WorldEulerConstraint(p,pelvis,[NaN;0;NaN],[NaN;0;NaN],tspan);

  ikproblem = InverseKinematics(p,q_nom,l_foot_con,l_foot_pose_con, ...
    r_foot_con,r_foot_pose_con,pelvis_con,pelvis_pose_con);
  ikproblem = ikproblem.setQ(ikoptions.Q);
  ikproblem = ikproblem.setSolverOptions('fmincon','Algorithm','sqp');
  ikproblem = ikproblem.setSolverOptions('fmincon','MaxIter',500);
  ikproblem = ikproblem.setSolverOptions('snopt','MajorIterationsLimit',ikoptions.SNOPT_MajorIterationsLimit);
  [qik,F,info,infeasible_cnstr_ik] = ikproblem.solve(q0);
  qs(:,i) = qik;
end

qtraj = PPTrajectory(foh(ts,qs));
v = p.constructVisualizer();

playback(v,qtraj,struct('slider',true));

keyboard;
save('data/atlas_step_qtraj.mat','qtraj');


