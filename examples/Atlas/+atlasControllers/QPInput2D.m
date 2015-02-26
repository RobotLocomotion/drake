classdef QPInput2D
  properties(Constant)
  end

  properties
    timestamp
    zmp_data
    support_data 
    body_motion_data 
    whole_body_data 
    param_set_name
  end

  methods
    function obj = QPInput2D()
      obj.timestamp = 0;
      obj.zmp_data = struct('A',  [zeros(2),eye(2); zeros(2,4)],... % COM state map % 16 d
                        'B', [zeros(2); eye(2)],... % COM input map % 4 d
                        'C', [eye(2),zeros(2)],... % ZMP state-output map % 4 d
                        'D', -0.89/9.81*eye(2),... % ZMP input-output map % 2 d
                        'x0', zeros(4,1),... % nominal state % 4 d
                        'y0', zeros(2,1),... % nominal output % 2 d
                        'u0', zeros(2,1),... % nominal input % 2 d
                        'R', zeros(2),... % input LQR cost % 4 d
                        'Qy', eye(2),... % output LQR cost % 4 d
                        'S', zeros(4),... % cost-to-go terms: x'Sx + x's1 + s2 % 16 d
                        's1', zeros(4,1),... % 4 d
                        's1dot', zeros(4,1),... % 4 d
                        's2', 0,... % 1 d
                        's2dot', 0); % 1 d

      % support_logic_map lets us specify the way the various forms of contact detection are combined. 
      % We do this by enumerating all possible binary inputs from the force sensor and the kinematic
      % contact detector. Then we have the following table:
      %
      % force  kin  output
      % 0      0    case1
      % 0      1    case2
      % 1      0    case3
      % 1      1    case4
      % 
      % The support_logic_map for each body is the vector [case1; case2; case3; case4];
      % For example, to force a body to be in support regardless of sensors, we would say:
      % support_logic_map = ones(4,1);
      % To allow support only if the force sensor is TRUE, we would say:
      % support_logic_map = [0;0;1;1];
      %
      obj.support_data = struct('body_id', {}, 'contact_pts', {}, 'support_logic_map', {}, 'mu', {}, 'contact_surfaces', {});
      
      obj.body_motion_data = struct('body_id', {},... % 3 d
                            'ts', {},... % 6 d
                            'coefs', {}); % 4 * 6 * 3 d
      obj.whole_body_data = struct('q_des', [],... % 34 d
                               'constrained_dof_mask', []); % 34 b
      obj.param_set_name = 'walking';
    end

    function msg = to_lcm(obj)
      msg = drake.lcmt_qp_controller_input();
      msg.timestamp = obj.timestamp;
      msg.zmp_data = drake.lcmt_zmp_data();
      for f = fieldnames(obj.zmp_data)'
        msg.zmp_data.(f{1}) = obj.zmp_data.(f{1});
      end
      nsupp = length(obj.support_data);
      msg.num_support_data = nsupp;
      msg.support_data = javaArray('drake.lcmt_support_data', nsupp);
      for j = 1:nsupp
        msg.support_data(j) = drake.lcmt_support_data();
        msg.support_data(j).body_id = obj.support_data(j).body_id;
        msg.support_data(j).num_contact_pts = size(obj.support_data(j).contact_pts, 2);
        msg.support_data(j).contact_pts = obj.support_data(j).contact_pts;
        msg.support_data(j).support_logic_map = logical(obj.support_data(j).support_logic_map);
        msg.support_data(j).mu = obj.support_data(j).mu;
        msg.support_data(j).contact_surfaces = obj.support_data(j).contact_surfaces;
      end

      nbod = length(obj.body_motion_data);
      msg.num_tracked_bodies = nbod;
      msg.body_motion_data = javaArray('drake.lcmt_body_motion_data', nbod);
      for j = 1:nbod
        msg.body_motion_data(j) = drake.lcmt_body_motion_data();
        msg.body_motion_data(j).body_id = obj.body_motion_data(j).body_id;
        msg.body_motion_data(j).ts = obj.body_motion_data(j).ts;
        msg.body_motion_data(j).coefs = obj.body_motion_data(j).coefs;
      end
      
      msg.whole_body_data = drake.lcmt_whole_body_data();
      msg.whole_body_data.num_positions = numel(obj.whole_body_data.q_des);
      msg.whole_body_data.q_des = obj.whole_body_data.q_des;
      msg.whole_body_data.constrained_dof_mask = obj.whole_body_data.constrained_dof_mask;
      msg.param_set_name = obj.param_set_name;
    end
  end
end



% 219 doubles, 148 bytes
