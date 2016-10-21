function collidingPointsTest(visualize)
  
  checkDependency('bullet');
  options.floating = true;
  urdf = fullfile('../../../../examples/Atlas/urdf/atlas_convex_hull.urdf');
  robot = RigidBodyManipulator(urdf);
  q = zeros(robot.num_positions,1);
  kinsol = robot.doKinematics(q);
  
  [x,y,z] = meshgrid(-1:.2:1, -1:.2:1, -1:.2:1);
  point_cloud = [reshape(x,1,[]);reshape(y,1,[]);reshape(z,1,[])];
  threshold = 0.05;
  
  cp = collidingPoints(robot, kinsol, point_cloud, threshold);
  is_in_collision = collidingPointsCheckOnly(robot, kinsol, point_cloud, threshold);
  
  cp_result = [60,61,62,73,181,182,183,302,303,304,423,424,425,534,544,545,...
    546,655,665,666,667,677,775,776,777,786,787,788,798,896,897,898,903,904,...
    905,906,907,908,909,910,911,912,914,915,916,917,918,919,920,921,922,923,...
    924,1007,1017,1018,1019,1028,1029,1030,1039,1040,1041,1128,1138,1139,1140,...
    1149,1150,1151,1160,1161,1162,1259,1260,1270,1271,1272,1282,1283];
  
  if ~is_in_collision
    error('collidingPointsCheckOnly must return true for this test')
  end
  
  if any(cp ~= cp_result)
    error('collidingPoints output is not correct')
  end
  
  
  if nargin < 1, visualize = false; end;
  if visualize
    checkDependency('lcmgl');
    v = robot.constructVisualizer();
    v.draw(0,q)
    
    lcmClient = LCMGLClient('point cloud');
    lcmClient.glColor3f(0, 1, 0);
    lcmClient.points(point_cloud(1,:), point_cloud(2,:), point_cloud(3,:))
    lcmClient.switchBuffers();
    
    lcmClient = LCMGLClient('colliding points');
    lcmClient.glColor3f(1, 0, 1);
    lcmClient.points(point_cloud(1,cp), point_cloud(2,cp), point_cloud(3,cp))
    lcmClient.switchBuffers();
  end

end
