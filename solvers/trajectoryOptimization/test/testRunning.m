function [sol,robot_vis,v,cdfkp] = testRunning(seed,stride_length,major_iteration_limit, suffix)
  checkDependency('lcmgl');
  if nargin < 1, seed = []; end
  if (nargin < 2 || isempty(stride_length)), stride_length = 2; end
  if (nargin < 3 || isempty(major_iteration_limit)), major_iteration_limit = 5; end
  if (nargin < 4 || isempty(suffix)), suffix = ''; end

  % Construct RigidBodyManipulator
  w = warning('off','Drake:RigidBody:SimplifiedCollisionGeometry');
  warning('off','Drake:RigidBody:NonPositiveInertiaMatrix');
  warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
  warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
  options.floating = true;
  options.terrain = RigidBodyFlatTerrain;
  atlas_urdf = [getDrakePath,'/examples/Atlas/urdf/atlas_convex_hull.urdf'];
  atlas_vis_urdf = [getenv('DRC_BASE'),'/software/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'];
  robot = RigidBodyManipulator(atlas_urdf,options);
  robot_vis = RigidBodyManipulator(atlas_vis_urdf,options);
  mu = 1.16; % rubber on rubber
  warning(w);

  % Create convenience variables
  nq = robot.getNumPositions();
  nv = robot.getNumVelocities();
  world = robot.findLinkInd('world');
  l_foot = robot.findLinkInd('l_foot');
  r_foot = robot.findLinkInd('r_foot');
  neck_idx = robot.getBody(robot.findJointInd('neck_ay')).dofnum;
  freeze_idx = [neck_idx];
  l_foot_bottom = robot.getBody(l_foot).getTerrainContactPoints();
  r_foot_bottom = robot.getBody(r_foot).getTerrainContactPoints();
  l_foot_toe    = robot.getBody(l_foot).getTerrainContactPoints('toe');
  r_foot_toe    = robot.getBody(r_foot).getTerrainContactPoints('toe');
  l_foot_heel   = robot.getBody(l_foot).getTerrainContactPoints('heel');
  r_foot_heel   = robot.getBody(r_foot).getTerrainContactPoints('heel');

  % Construct visualization tools
  v = constructVisualizer(robot_vis);
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'asdf');

  % Load nominal data
  nomdata = load([getenv('DRC_BASE'),'/software/control/matlab/data/atlas_fp.mat']);
  qstar = nomdata.xstar(1:nq);
  com_star = robot.getCOM(qstar);
  vstar = zeros(nv,1);
  v.draw(0,qstar);
  q0 = qstar;
  q0(1) = -stride_length/4;
  qf = q0;
  qf(1) = stride_length/4;

  % Set up time limits
  N = 18;
  tf_range = [0.2,2];
  h_min = 0.01; h_max_stance = 0.05; h_max_flight = 0.3;

  % Set up cost variables
  q_nom = bsxfun(@times,qstar,ones(1,N));
  Q = eye(nq);
  Q(1,1) = 0;
  Q(2,2) = 0;
  Q(6,6) = 0;
  Qv = eye(nq);
  Q_comddot = eye(3);
  Q_contact_force = eye(3);

  % Create collision avoidance constraints
  min_distance = 0.03;
  % Consider all bodies
  min_distance_constraint.flight = MinDistanceConstraint(robot,min_distance,[-inf,inf]);
  % Ignore left foot
  active_collision_options.body_idx = setdiff(1:robot.getNumBodies(),l_foot);
  min_distance_constraint.stance = MinDistanceConstraint(robot,min_distance,[-inf,inf],active_collision_options);

  % Set up linearized friction cone edges
  num_edges = 3;
  FC_angles = linspace(0,2*pi,num_edges+1);FC_angles(end) = [];
  FC_axis = [0;0;1];
  FC_perp1 = rotx(pi/2)*FC_axis;
  FC_perp2 = cross(FC_axis,FC_perp1);
  FC_edge = bsxfun(@plus,FC_axis,mu*(bsxfun(@times,cos(FC_angles),FC_perp1) + ...
    bsxfun(@times,sin(FC_angles),FC_perp2)));
  FC_edge = robot.getMass()*norm(robot.getGravity)*bsxfun(@rdivide,FC_edge,sqrt(sum(FC_edge.^2,1)));

  % Create stance constraints
  in_stance.toe = 4:15;
  in_stance.heel = 8:11;
  in_stance.total = union(in_stance.toe,in_stance.heel);
  in_flight = setdiff(1:N,in_stance.total);
  contact_wrench_struct(1).active_knot = in_stance.toe;
  contact_wrench_struct(1).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_toe,FC_edge);
  contact_wrench_struct(2).active_knot = in_stance.heel;
  contact_wrench_struct(2).cw = ...
    LinearFrictionConeWrench(robot,l_foot,l_foot_heel,FC_edge);
  % Kinematic constraints
  lb = repmat([NaN; NaN; 0],1,2);
  ub = repmat([NaN; NaN; 0],1,2);
  position_cnstr.toe   = WorldPositionConstraint(robot,l_foot,l_foot_toe,lb,ub);
  position_cnstr.heel   = WorldPositionConstraint(robot,l_foot,l_foot_heel,lb,ub);
  stationary_cnstr.toe = WorldFixedPositionConstraint(robot,l_foot,l_foot_toe);
  stationary_cnstr.heel = WorldFixedPositionConstraint(robot,l_foot,l_foot_heel);

  % Create trajectory optimization
  cdfkp = ComDynamicsFullKinematicsPlanner(robot,N,tf_range,Q_comddot,Qv,Q,q_nom,Q_contact_force,contact_wrench_struct);

  % Add Timestep bounds
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_flight)-1,1),h_max_flight*ones(numel(in_flight)-1,1)),cdfkp.h_inds(in_flight(1:end-1)));
  cdfkp = cdfkp.addBoundingBoxConstraint(BoundingBoxConstraint(h_min*ones(numel(in_stance.total)-1,1),h_max_stance*ones(numel(in_stance.total)-1,1)),cdfkp.h_inds(in_stance.total(1:end-1)));
  
  % Add initial contition constraints
  cdfkp = cdfkp.addConstraint(ConstantConstraint([-stride_length/4;0;0]), ...
    [cdfkp.q_inds(1:2,1);cdfkp.comdot_inds(3,1)]);

  % Add final condition constraints
  cdfkp = cdfkp.addConstraint(ConstantConstraint([stride_length/4]), ...
    [cdfkp.q_inds(1,end)]);

  % Immobilize specified joints
  cdfkp = cdfkp.addConstraint(ConstantConstraint(repmat(q0(freeze_idx),N,1)),cdfkp.q_inds(freeze_idx,:));

  % Add periodicity constraints
  half_periodic_constraint = halfPeriodicConstraint(robot);
  cdfkp = cdfkp.addConstraint(half_periodic_constraint,cdfkp.q_inds(:,[1,end]));
  cdfkp = cdfkp.addConstraint(half_periodic_constraint,cdfkp.v_inds(:,[1,end]));
  cdfkp = cdfkp.addConstraint(LinearConstraint(0,0,[1,-1]),cdfkp.v_inds(1,[1,end]));
  %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(2,1),zeros(2,1),[eye(2),-eye(2)]), ...
                              %cdfkp.com_inds(2:3,[1,end]));
  %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,-1])]), ...
                              %cdfkp.comdot_inds(1:3,[1,end]));
  %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([-1,1,-1])]), ...
                              %cdfkp.comddot_inds(1:3,[1,end]));
  %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]), ...
                              %cdfkp.H_inds(1:3,[1,end]));
  %cdfkp = cdfkp.addConstraint(LinearConstraint(zeros(3,1),zeros(3,1),[eye(3),diag([1,-1,1])]),cdfkp.Hdot_inds(1:3,[1,end]));

  % Add stance kinematic constraints
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.toe, ...
    {num2cell(in_stance.toe)});
  cdfkp = cdfkp.addRigidBodyConstraint(position_cnstr.heel, ...
    {num2cell(in_stance.heel)});
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.toe, ...
    {in_stance.toe});
  cdfkp = cdfkp.addRigidBodyConstraint(stationary_cnstr.heel, ...
    {in_stance.heel});

  % Add collision avoidance constraints
  cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.flight,num2cell(setdiff(1:N,[in_stance.toe,in_stance.heel])));
  cdfkp = cdfkp.addRigidBodyConstraint(min_distance_constraint.stance,num2cell(union(in_stance.toe,in_stance.heel)));
  
  % TODO: Set up seed
  if isempty(seed)
    x_seed = zeros(cdfkp.num_vars,1);
    x_seed(cdfkp.h_inds) = (h_min+h_max_flight+h_max_stance)/2;
    x_seed(cdfkp.q_inds(:)) = reshape(linspacevec(q0,qf,N),[],1);
  else
    x_seed = seed.x_sol;
  end
  
  % Set up solver options
  cdfkp = cdfkp.setSolverOptions('snopt','iterationslimit',1e6);
  cdfkp = cdfkp.setSolverOptions('snopt','majoriterationslimit',major_iteration_limit);
  cdfkp = cdfkp.setSolverOptions('snopt','majorfeasibilitytolerance',1e-5);
  cdfkp = cdfkp.setSolverOptions('snopt','majoroptimalitytolerance',1e-5);
  cdfkp = cdfkp.setSolverOptions('snopt','superbasicslimit',2000);
  cdfkp = cdfkp.setSolverOptions('snopt','linesearchtolerance',0.99);
  cdfkp = cdfkp.setSolverOptions('snopt','print',sprintf('snopt_%s.out',suffix));
  
  % Solve trajectory optimization
  tic
  %profile on;
  [x_sol,F,info] = cdfkp.solve(x_seed);
  %profile off;
  toc
  
  % Parse trajectory optimization output
  sol.x_sol = x_sol;
  sol.q = reshape(x_sol(cdfkp.q_inds(:)),nq,N);
  sol.v = reshape(x_sol(cdfkp.v_inds(:)),nq,N);
  sol.h = reshape(x_sol(cdfkp.h_inds),1,[]);
  sol.t = cumsum([0 sol.h]);
  sol.com = reshape(x_sol(cdfkp.com_inds),3,[]);
  sol.comdot = reshape(x_sol(cdfkp.comdot_inds),3,[]);
  sol.comddot = reshape(x_sol(cdfkp.comddot_inds),3,[]);
  sol.H = reshape(x_sol(cdfkp.H_inds),3,[]);
  sol.Hdot = reshape(x_sol(cdfkp.Hdot_inds),3,[])*cdfkp.torque_multiplier;
  sol.lambda = cell(2,1);
  for i = 1:numel(cdfkp.lambda_inds)
    sol.lambda{i} = reshape(x_sol(cdfkp.lambda_inds{i}),size(cdfkp.lambda_inds{i},1),[],N);
  end
  sol.xtraj= PPTrajectory(foh(sol.t,[sol.q;sol.v]));
  sol.xtraj= sol.xtraj.setOutputFrame(robot_vis.getStateFrame);

  % Save results
  save(sprintf('results_%s',suffix),'sol');
end

function half_periodic_constraint = halfPeriodicConstraint(robot)
  num_symmetry = 12;
  num_equal = 8;
  nq = robot.getNumPositions();

  symmetric_matrix = zeros(2*num_symmetry,2*nq);
  equal_matrix = zeros(num_equal,2*nq);
  initial_indices = 1:nq;
  final_indices   = nq+(1:nq);

  function sym_mat = addSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 -1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 -1];
  end

  function sym_mat = addAntiSymmetricPair(sym_mat,rows,idx1,idx2)
    sym_mat(rows(1),[initial_indices(idx1) final_indices(idx2)]) = [1 1];
    sym_mat(rows(2),[initial_indices(idx2) final_indices(idx1)]) = [1 1];
  end

  function eq_mat = addEquality(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 -1];
  end

  function eq_mat = addOpposite(eq_mat,row,idx)
    eq_mat(row,[initial_indices(idx) final_indices(idx)]) = [1 1];
  end

  l_arm_usy_idx = robot.getBody(robot.findJointInd('l_arm_usy')).dofnum;
  r_arm_usy_idx = robot.getBody(robot.findJointInd('r_arm_usy')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,1:2,l_arm_usy_idx,r_arm_usy_idx);

  l_arm_shx_idx = robot.getBody(robot.findJointInd('l_arm_shx')).dofnum;
  r_arm_shx_idx = robot.getBody(robot.findJointInd('r_arm_shx')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,3:4,l_arm_shx_idx,r_arm_shx_idx);

  l_arm_ely_idx = robot.getBody(robot.findJointInd('l_arm_ely')).dofnum;
  r_arm_ely_idx = robot.getBody(robot.findJointInd('r_arm_ely')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,5:6,l_arm_ely_idx,r_arm_ely_idx);

  l_arm_elx_idx = robot.getBody(robot.findJointInd('l_arm_elx')).dofnum;
  r_arm_elx_idx = robot.getBody(robot.findJointInd('r_arm_elx')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,7:8,l_arm_elx_idx,r_arm_elx_idx);

  l_arm_uwy_idx = robot.getBody(robot.findJointInd('l_arm_uwy')).dofnum;
  r_arm_uwy_idx = robot.getBody(robot.findJointInd('r_arm_uwy')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,9:10,l_arm_uwy_idx,r_arm_uwy_idx);

  l_arm_mwx_idx = robot.getBody(robot.findJointInd('l_arm_mwx')).dofnum;
  r_arm_mwx_idx = robot.getBody(robot.findJointInd('r_arm_mwx')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,11:12,l_arm_mwx_idx,r_arm_mwx_idx);

  l_leg_hpz_idx = robot.getBody(robot.findJointInd('l_leg_hpz')).dofnum;
  r_leg_hpz_idx = robot.getBody(robot.findJointInd('r_leg_hpz')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,13:14,l_leg_hpz_idx,r_leg_hpz_idx);

  l_leg_hpx_idx = robot.getBody(robot.findJointInd('l_leg_hpx')).dofnum;
  r_leg_hpx_idx = robot.getBody(robot.findJointInd('r_leg_hpx')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,15:16,l_leg_hpx_idx,r_leg_hpx_idx);

  l_leg_hpy_idx = robot.getBody(robot.findJointInd('l_leg_hpy')).dofnum;
  r_leg_hpy_idx = robot.getBody(robot.findJointInd('r_leg_hpy')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,17:18,l_leg_hpy_idx,r_leg_hpy_idx);

  l_leg_kny_idx = robot.getBody(robot.findJointInd('l_leg_kny')).dofnum;
  r_leg_kny_idx = robot.getBody(robot.findJointInd('r_leg_kny')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,19:20,l_leg_kny_idx,r_leg_kny_idx);

  l_leg_akx_idx = robot.getBody(robot.findJointInd('l_leg_akx')).dofnum;
  r_leg_akx_idx = robot.getBody(robot.findJointInd('r_leg_akx')).dofnum;
  symmetric_matrix = addAntiSymmetricPair(symmetric_matrix,21:22,l_leg_akx_idx,r_leg_akx_idx);

  l_leg_aky_idx = robot.getBody(robot.findJointInd('l_leg_aky')).dofnum;
  r_leg_aky_idx = robot.getBody(robot.findJointInd('r_leg_aky')).dofnum;
  symmetric_matrix = addSymmetricPair(symmetric_matrix,23:24,l_leg_aky_idx,r_leg_aky_idx);

  base_y = findJointIndices(robot,'base_y'); base_y = base_y(1);
  equal_matrix = addOpposite(equal_matrix,1,base_y);

  base_z = findJointIndices(robot,'base_z');
  equal_matrix = addEquality(equal_matrix,2,base_z);

  base_roll = findJointIndices(robot,'base_roll');
  equal_matrix = addOpposite(equal_matrix,3,base_roll);

  base_pitch = findJointIndices(robot,'base_pitch');
  equal_matrix = addEquality(equal_matrix,4,base_pitch);

  base_yaw = findJointIndices(robot,'base_yaw');
  equal_matrix = addOpposite(equal_matrix,5,base_yaw);

  back_bkz = findJointIndices(robot,'back_bkz');
  equal_matrix = addOpposite(equal_matrix,6,back_bkz);

  back_bky = findJointIndices(robot,'back_bky');
  equal_matrix = addEquality(equal_matrix,7,back_bky);

  back_bkx = findJointIndices(robot,'back_bkx');
  equal_matrix = addOpposite(equal_matrix,8,back_bkx);

  lb = zeros(2*num_symmetry+num_equal,1);
  ub = lb;
  half_periodic_constraint = LinearConstraint(lb,ub,[symmetric_matrix;equal_matrix]);
end

