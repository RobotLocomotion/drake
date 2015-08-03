function multiContactTest
  checkDependency('bullet');
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'bullet_collision_closest_points_test');
  dt = 0.01;
  
  options.enable_fastqp = false;
  options.use_bullet = true;
  options.terrain = RigidBodyFlatTerrain();
  options.multiple_contacts = true;
  p = TimeSteppingRigidBodyManipulator([], dt, options);
    
  padding = 0.04;
  q0 = [0;0;0.5+25*padding;0;0;0];
  q1 = [0;0;1.5+50*padding;0;0;pi/2];
  q2 = [0;0.75;2.5+100*padding;0;0;0];
  q3 = [0;-0.75;2.5+100*padding;0;0;0];
  options.floating = true;
  options.terrain = [];
  
  for i = 1:4
    p = p.addRobotFromURDF('FallingBrick.urdf', [], [], options);
  end
  
  xtraj = p.simulate([0 3], [q0;q1;q2;q3;0*q0;0*q1;0*q2;0*q3]);
  v = p.constructVisualizer();
  for t = xtraj.getBreaks
    xt = xtraj.eval(t);
    qt = xt(1:4*size(q0,1));
    kinsol = p.doKinematics(qt);
    [phi, normal, xA, xB, idxa, idxb] = p.collisionDetect(kinsol, true);
    v.draw(t,qt);
    debugLCMGL(p, v, kinsol, phi, normal, xA, xB, idxa, idxb)
    pause(dt)
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
    lcmgl.line3(xA_in_world(1), xA_in_world(2), xA_in_world(3), ...
                xB_in_world(1), xB_in_world(2), xB_in_world(3));
    
    lcmgl.glColor3f(.7,.7,.7); % gray
    end
    
    lcmgl.switchBuffers();
end

end


