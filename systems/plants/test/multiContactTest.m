function multiContactTest
  checkDependency('bullet');
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');
  dt = 0.01;
  
  options.enable_fastqp = false;
  options.use_bullet = true;
  options.terrain = RigidBodyFlatTerrain();
  p = TimeSteppingRigidBodyManipulator([], dt, options);
    
  padding = 0.02;
  padding2 = 0.02;
  q0 = [0;0;0.5+padding;0;0;0];
  q1 = [0;0.0;1.5+padding+padding2;0;0;pi/2];
  options.floating = true;
  options.terrain = [];
  
  %p = p.addRobotFromURDF('FallingBrickContactPoints.urdf', [], [], options);
  %p = p.addRobotFromURDF('FallingBrickContactPoints.urdf', [], [], options);
  
  p = p.addRobotFromURDF('FallingBrick.urdf', [], [], options);
  p = p.addRobotFromURDF('FallingBrick.urdf', [], [], options);
  
  xtraj = p.simulate([0 3], [q0;q1;0*q0;0*q1])
  v = p.constructVisualizer();
  for t = xtraj.getBreaks
    xt = xtraj.eval(t);
    qt = xt(1:size(q0,1)+size(q1,1));
    kinsol = p.doKinematics(qt);
    [phi, normal, xA, xB, idxa, idxb] = p.collisionDetect(kinsol, true);
    v.draw(t,qt);
    debugLCMGL(p, v, kinsol, phi, normal, xA, xB, idxa, idxb)
  end
  
function debugLCMGL(r,v,kinsol,phi,normal,xA,xB,idxA,idxB)

    for i = 1:size(xA,2)
    xA_in_world = forwardKin(r, kinsol, idxA(i), xA(:,i));
    xB_in_world = forwardKin(r, kinsol, idxB(i), xB(:,i));
    lcmgl.glColor3f(1,0,0); % red
    lcmgl.sphere(xA_in_world,.02,20,20);

    lcmgl.glColor3f(0,1,0); % green
    lcmgl.sphere(xB_in_world,.02,20,20);

    lcmgl.glColor3f(0,0,0); % black
    lcmgl.glLineWidth(5); % black
    lcmgl.line3(xA_in_world(1),xA_in_world(2),xA_in_world(3), ...
                xB_in_world(1),xB_in_world(2),xB_in_world(3));
    
    lcmgl.glColor3f(.7,.7,.7); % gray
    end
    
    lcmgl.switchBuffers();

    %pause(0.05);
end

end


