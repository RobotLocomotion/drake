function body_vdot = statelessBodyMotionControl(robot, x, body_id, body_pose_des, body_v_des, body_vdot_des, params)

nq = robot.getNumPositions();
q = x(1:nq);
qd = x(nq+1:end);
kinsol = doKinematics(robot,q,false,true,qd);

lcmgl = LCMGLClient(sprintf('link_%d_desired', body_id));
lcmgl.sphere(body_pose_des(1:3), 0.03, 20, 20);
lcmgl.switchBuffers();

% TODO: this should be updated to use quaternions/spatial velocity
[p,J] = forwardKin(robot,kinsol,body_id,[0;0;0],1); 

err = [body_pose_des(1:3)-p(1:3);angleDiff(p(4:end),body_pose_des(4:end))];

body_vdot = params.Kp.*err + params.Kd.*(body_v_des-J*qd) + body_vdot_des; 

end