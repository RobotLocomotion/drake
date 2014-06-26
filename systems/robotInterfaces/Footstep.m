classdef Footstep
% A data structure for a single footstep position.
% The pose of the footstep is assumed to be expressed as the [x,y,z,r,p,y] pose
% of the center sole of the foot, which is the location expressed by the Drake
% frame given by frame_id
  properties
    pos
    id
    frame_id
    is_in_contact
    pos_fixed
    terrain_pts
    infeasibility
    walking_params
  end

  methods
    function obj = Footstep(pos, id, frame_id, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params)
      obj.pos = pos;
      obj.id = id;
      obj.frame_id = frame_id;
      obj.is_in_contact = is_in_contact;
      obj.pos_fixed = pos_fixed;
      obj.terrain_pts = terrain_pts;
      obj.infeasibility = infeasibility;
      obj.walking_params = walking_params;
    end

    function msg = to_footstep_t(obj, biped)
      msg = drc.footstep_t();
      T = inv(biped.getFrame(obj.frame_id).T);
      pos = zeros(6,1);
      xyz1 = T * [obj.pos(1:3);1];
      pos(1:3) = xyz1(1:3);
      pos(4:6) = rotmat2rpy(rpy2rotmat(obj.pos(4:6)) * T(1:3,1:3));
      msg.pos = encodePosition3d(pos);
      msg.id = obj.id;
      if obj.frame_id == biped.foot_frame_id.right
        msg.is_right_foot = true;
      elseif obj.frame_id == biped.foot_frame_id.left
        msg.is_right_foot = false;
      else
        error('Drake:Footstep:InvalidFrameID', 'Invalid Drake frame ID');
      end
      msg.is_in_contact = obj.is_in_contact;
      msg.fixed_x = obj.pos_fixed(1);
      msg.fixed_y = obj.pos_fixed(2);
      msg.fixed_z = obj.pos_fixed(3);
      msg.fixed_roll = obj.pos_fixed(4);
      msg.fixed_pitch = obj.pos_fixed(5);
      msg.fixed_yaw = obj.pos_fixed(6);
      msg.num_terrain_pts = size(obj.terrain_pts, 2);
      if msg.num_terrain_pts > 0
        msg.terrain_path_dist = obj.terrain_pts(1,:);
        msg.terrain_height = obj.terrain_pts(2,:);
      end
      msg.infeasibility = obj.infeasibility;
      msg.params = populateLCMFields(drc.footstep_params_t(), obj.walking_params);
    end
  end

  methods(Static=true)
    function footstep = from_footstep_t(msg, biped)
      id = msg.id;
      if msg.is_right_foot
        frame_id = biped.foot_frame_id.right;
      else
        frame_id = biped.foot_frame_id.left;
      end
      msg_pos = decodePosition3d(msg.pos);
      T = biped.getFrame(frame_id).T;
      pos = zeros(6,1);
      xyz1 = T * [msg_pos(1:3);1];
      pos(1:3) = xyz1(1:3);
      pos(4:6) = rotmat2rpy(rpy2rotmat(msg_pos(4:6)) * T(1:3,1:3));
      is_in_contact = msg.is_in_contact;
      pos_fixed = [msg.fixed_x;
                   msg.fixed_y;
                   msg.fixed_z;
                   msg.fixed_roll;
                   msg.fixed_pitch;
                   msg.fixed_yaw];
      terrain_pts = [reshape(msg.terrain_path_dist, 1, []);
                     reshape(msg.terrain_height, 1, []);];
      infeasibility = msg.infeasibility;
      walking_params = msg.params;
      footstep = Footstep(pos, id, frame_id, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
    end
  end
end




