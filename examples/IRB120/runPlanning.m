function runPlanning()

r=RigidBodyManipulator('irb_120.urdf',struct('floating',true));  

v=r.constructVisualizer();

% default starting pose
q0 = [-0.5, 0, 1.5, pi/2, pi/2, 0,0,0,0,0,0,0]'; 

%pos_final = [-0.5, 0, 1.015]';
pos_final = [0, 0, 1.05]';
gripper_idx = findLinkInd(r,'link_6');
gripper_pt = [-0.04,0,0.1]';
grasp_orient = angle2quat(0,pi,0)';

T = 1;
N = 2;
Allcons = cell(0,1);

gripper_cons = WorldPositionConstraint(r, gripper_idx,gripper_pt,pos_final,pos_final,[T,T]);

tol = 0;
r_gripper_cons_orient = WorldQuatConstraint(r,gripper_idx,grasp_orient,tol,[0.1*T,T]);

Allcons{end+1} = r_gripper_cons_orient;

Allcons{end+1} = gripper_cons;
for i=1:6
  Allcons{end+1} = PostureChangeConstraint(r,i,0,0); % base
end


% compute seeds
q_start_nom = q0;

[q_end_nom,snopt_info_ik,infeasible_constraint_ik] = inverseKin(r,q_start_nom,q_start_nom,...
Allcons{:});
qtraj_guess = PPTrajectory(foh([0 T],[q_start_nom, q_end_nom]));

t_vec = linspace(0,T,N);

% do IK
[xtraj,snopt_info,infeasible_constraint] = inverseKinTraj(r,...
t_vec,qtraj_guess,qtraj_guess,...
Allcons{:});

q_end = xtraj.eval(xtraj.tspan(end));

% do visualize
if snopt_info <= 10
  v.playback(xtraj);
end

if snopt_info > 10
  error('IK fail snopt_info: %d\n', snopt_info);
end

