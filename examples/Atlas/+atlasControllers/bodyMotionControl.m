function body_vdot = bodyMotionControl(robot, x, body_id, body_pose_des, body_v_des, body_vdot_des, params)
% NOTEST
% Run PD on a desired body trajectory to compute body accelerations, which
% we will hand to the QP. This mirrors instantaneousBodyMotionControlmex.cpp
% @param robot 
% @param x current state vector
% @param body_id the ID of the body (1-indexed)
% @param body_pose_des the desired 6-DOF pose of the body
% @param body_v_des the desired 6-DOF velocity
% @param body_vdot_des the desired 6-DOF acceleration
% @param params an element of the body_motion field of an atlasParams.* object
DEBUG = false;

nq = robot.getNumPositions();
q = x(1:nq);
qd = x(nq+1:end);
kinsol = doKinematics(robot,q,false,true,qd);

if DEBUG
  lcmgl = LCMGLClient(sprintf('link_%d_desired', body_id));
  lcmgl.sphere(body_pose_des(1:3), 0.03, 20, 20);
  lcmgl.switchBuffers();
end

% TODO: this should be updated to use quaternions/spatial velocity
[p,J] = forwardKin(robot,kinsol,body_id,[0;0;0],1); 

err = [body_pose_des(1:3)-p(1:3);angleDiff(p(4:end),body_pose_des(4:end))];

body_vdot = params.Kp.*err + params.Kd.*(body_v_des-J*qd) + body_vdot_des; 

end