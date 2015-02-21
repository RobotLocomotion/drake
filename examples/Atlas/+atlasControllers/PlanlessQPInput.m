classdef PlanlessQPInput2D
  properties(Constant)
    num_support_bodies = 30;
    support_body_ids = 1:30;
    contact_groups_per_body = 4;
    num_positions = 34;
    num_body_motions = 3;
  end

  properties
    zmp_data = struct('A',  [zeros(2),eye(2); zeros(2,4)],... % COM state map % 16 d
                      'B', [zeros(2); eye(2)],... % COM input map % 4 d
                      'C', [eye(2),zeros(2)],... % ZMP state-output map % 4 d
                      'D', -0.94/9.81*eye(2),... % ZMP input-output map % 2 d
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
    support_data = struct('group_mask', false(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body), ... % 28 * 4 b
                          'right_toe_off', false,... % 1 b
                          'left_toe_off', false,... % 1 b
                          'mu', 1); % 1 d
    
    bodies_data = struct('body_id', cell(1, PlanlessQPInput2D.num_body_motions),... % 3 d
                          'ts', cell(1, PlanlessQPInput2D.num_body_motions),... % 6 d
                          'coefs', cell(1, PlanlessQPInput2D.num_body_motions),... % 4 * 6 * 3 d
                          'Kp', cell(1, PlanlessQPInput2D.num_body_motions),... % 6 * 3 d
                          'Kd', cell(1, PlanlessQPInput2D.num_body_motions)); % 6 * 3 d
    whole_body_data = struct('q_des', zeros(PlanlessQPInput2D.num_positions, 1),... % 34 d
                             'Kp', zeros(PlanlessQPInput2D.num_positions, 1),...
                             'Kd', zeros(PlanlessQPInput2D.num_positions, 1),...
                             'constrained_dof_mask', false(PlanlessQPInput2D.num_positions, 1)); % 34 b
  end
end



% 219 doubles, 148 bytes
