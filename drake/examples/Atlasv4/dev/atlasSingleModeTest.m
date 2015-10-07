options_new.mode_seq_vec = options.mode_seq_vec(:,1);
options_new.t_span_mode = {options.t_span_mode{1} - options.t_span_mode{1}(1)};
options_new.N_vec = options.N_vec(1);
options_new.T_span = options.t_span_mode{1} - options.t_span_mode{1}(1);

options_new.x0_delta = [0;0;0;.1;.1;.1;.1;.2;.2;1;0.1;5*ones(9,1)];
options_new.xf_delta = [0;0;0;.1;.1;.1;.1;.2;.2;1;0.1;5*ones(9,1)];

[xtraj_,utraj_,z_,F_,info_,traj_opt_] = hybridTrajOptPlanarAtlas(p,xtraj{1},utraj{1},ltraj,[],options_new);
