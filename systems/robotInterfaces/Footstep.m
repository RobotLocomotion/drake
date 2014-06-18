classdef Footstep
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
      msg.pos = encodePosition3d(obj.pos);
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
      msg.params = obj.walking_params;
    end
  end

  methods(Static=true)
    function footstep = from_footstep_t(msg, biped)
      pos = decodePosition3d(msg.pos);
      id = msg.id;
      if msg.is_right_foot
        frame_id = biped.foot_frame_id.right;
      else
        frame_id = biped.foot_frame_id.left;
      end
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
      footstep = Footstep(biped, pos, id, frame_id, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
    end
  end
end




