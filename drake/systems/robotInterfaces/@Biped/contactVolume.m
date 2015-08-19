function [contact_length, contact_width] = contactVolume(biped, swing1, swing2)
% Find the effective length and width of the prism swept out by the robot's foot as it moves
% from swing1 to swing2. 
% @param biped a Biped
% @param swing1 a Footstep
% @param swing2 a Footstep

xhat = swing2.pos(1:2) - swing1.pos(1:2);
xhat = xhat / norm(xhat);
yhat = rotmat(pi/2) * xhat;
xhat = [xhat; 0];
yhat = [yhat; 0];
% zhat = [0;0;1];

foot_body = biped.getBody(biped.getFrame(swing1.frame_id).body_ind);

T_sole_to_foot = biped.getFrame(swing1.frame_id).T;
contact_points_in_foot = foot_body.getTerrainContactPoints();
T_swing1_sole_to_world = [rpy2rotmat(swing1.pos(4:6)),swing1.pos(1:3); zeros(1, 3), 1];
T_swing1_foot_to_world = T_swing1_sole_to_world/T_sole_to_foot;
swing1_contact_points_in_world = T_swing1_foot_to_world(1:3,:) * ...
  [contact_points_in_foot; ones(1,size(contact_points_in_foot,2))];

T_swing2_sole_to_world = [rpy2rotmat(swing2.pos(4:6)),swing2.pos(1:3); zeros(1, 3), 1];
T_swing2_foot_to_world = T_swing2_sole_to_world/T_sole_to_foot;
swing2_contact_points_in_world = T_swing2_foot_to_world(1:3,:) * ...
  [contact_points_in_foot; ones(1,size(contact_points_in_foot,2))];

contact_width = max([max(yhat' * swing1_contact_points_in_world) - min(yhat' * swing1_contact_points_in_world),...
                      max(yhat' * swing2_contact_points_in_world) - min(yhat' * swing2_contact_points_in_world)]);

contact_length = max([max(xhat' * swing1_contact_points_in_world) - min(xhat' * swing1_contact_points_in_world),...
                      max(xhat' * swing2_contact_points_in_world) - min(xhat' * swing2_contact_points_in_world)]);

end