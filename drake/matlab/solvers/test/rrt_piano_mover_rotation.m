function rrt_piano_mover_rotation(n_obstacles, planning_mode, n_smoothing_passes)
  w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
  %rng(2)
  if nargin < 1 || isempty(n_obstacles)
    n_obstacles = 40;
  end
  if nargin < 2 || isempty(planning_mode), planning_mode = 'rrt'; end
  if nargin < 3 || isempty(n_smoothing_passes), n_smoothing_passes = 10; end
  urdf = fullfile(getDrakePath, 'matlab', 'systems', 'plants', 'test', 'FallingBrick.urdf');
  options.floating = true;
  r = RigidBodyManipulator(urdf, options);
  box_size = [1;1;1];
  xyz_min = [-5; -5; -5];
  xyz_max = [5; 5; 5];
  robot_geom = RigidBodyBox([1, 5, 0.5], [0;2.5;0], [0;pi/2;0]);
  r = r.addGeometryToBody(2, robot_geom);
  r = r.compile();
  xyz0 = [0; 0; 0];
  q0 = rpy2quat([0; 0; 0]);
  qf = rpy2quat([2*pi/3; 0; 0]);
  rpy0 = quat2rpy(q0);
  rpyf = quat2rpy(qf);

  prob = MotionPlanningProblem(3);
  min_distance = 0.2;
  i = 0;
  while i < n_obstacles
    xyz = xyz_min + rand(3,1).*(xyz_max - xyz_min);
    rpy = uniformlyRandomRPY();
    obstacle = RigidBodyBox(box_size, xyz, rpy);
    r_new = r.addGeometryToBody('world', obstacle);
    r_new = r_new.compile();
    if all(collisionConstraint(r_new, q0) > min_distance) && ...
       all(collisionConstraint(r_new, qf) > min_distance)
      r = r_new;
      i = i + 1;
    end
  end
  r = r.compile();

  TA = SO3MotionPlanningTree();
  collision_constraint = FunctionHandleConstraint(min_distance,Inf,4,@(q)collisionConstraint(r, q),-2);
  TA = TA.addConstraint(collision_constraint);
  v = r.constructVisualizer();
  v.draw(0,[xyz0; rpy0]);
  TA.max_edge_length = 0.01;
  TA.max_length_between_constraint_checks = 0.001;
  TA.visualization_point = [0; 5; 0];
  TA.visualization_origin = xyz0;

  TB = TA;

  TA = TA.setLCMGL('TA',[1,0,0]);
  TB = TB.setLCMGL('TB',[0,0,1]);


  options.display_after_every = 100;
  options.visualize = true;
  rrt_timer = tic;
  switch planning_mode
    case 'rrt'
      [TA, path_ids_A, info] = TA.rrt(q0, qf, options);
    case 'rrt_connect'
      [TA, path_ids_A, info] = TA.rrtConnect(q0, qf, TB, options);
  end
  rrt_time = toc(rrt_timer);
  fprintf('Timing:\n');
  fprintf('  RRT:       %5.2f s\n', rrt_time);
  if info == 1
    smoothing_timer = tic;
    [TA_smooth, id_last] = TA.recursiveConnectSmoothing(path_ids_A, n_smoothing_passes);
    path_ids_A = TA_smooth.getPathToVertex(id_last);
    smoothing_time = toc(smoothing_timer);
    fprintf('  Smoothing: %5.2f s\n', smoothing_time);

    TA_smooth = TA_smooth.setLCMGL('TA_smooth', TA_smooth.line_color);
    %drawTree(TA);
    %drawTree(TB);
    drawPath(TA_smooth, path_ids_A);

    %q_path = extractPath(TA_smooth, path_ids_A, TB_smooth, path_ids_B);
    q_path = extractPath(TA_smooth, path_ids_A);
    path_length = size(q_path,2);
    xyz_path = repmat(xyz0, 1, path_length);
    rpy_path = NaN(3, path_length);
    for i = 1:path_length
      rpy_path(:,i) = quat2rpy(q_path(:,i));
    end
    traj = PPTrajectory(foh(linspace(0,10,path_length), [xyz_path; rpy_path]));
    traj = traj.setOutputFrame(r.getPositionFrame());
    %v.playback(traj, struct('slider', true));
    v.playback(traj);
  end

  function c = collisionConstraint(r, q)
    rpy = quat2rpy(q);
    phi = r.collisionDetect([xyz0; rpy]);
    c = min(phi);
  end

  warning(w);
end
