classdef HybridRigidBodyMode < RigidBodyManipulator
% Reimplements the dynamics method of RBM with support for inequality
% constraints

  methods
    function obj = HybridRigidBodyMode(urdf_filename,options)
      if (nargin<4) options=struct(); end
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
      obj = obj@RigidBodyManipulator(urdf_filename,options);
      warning(w);
    end
    
    function xdot = dynamics(obj,t,x,u)
      q = x(1:obj.num_positions);
      v = x(obj.num_positions+1:end);
      qd = vToqdot(obj, q) * v;

      [H,C,B] = manipulatorDynamics(obj,q,v);
      Hinv = inv(H);
      if (obj.num_u>0) tau=B*u - C; else tau=-C; end

      if obj.num_velocity_constraints>0,
        error('not implemented yet');
      end

      % solve for constraint force with 
      %  min_f ||f||^2 
      %    subject to \phiddot_i >= 0 for all \phi_i <= lb_i
      %    and \phiddot_i <= 0 for all \phi_i >= ub
      %  todo: consider adding stabilization terms back in, too
      
      nf = 0;

      phi=[]; J=[]; dJ = []; lb=[]; ub=[];
      constraint_ids = [obj.joint_limit_constraint_id,obj.position_constraint_ids];
      
      for i=1:numel(constraint_ids)
        [this_phi,this_J, this_dJ] = obj.state_constraints{constraint_ids(i)}.eval(q);
        lb = [lb; obj.state_constraints{constraint_ids(i)}.lb];
        ub = [ub; obj.state_constraints{constraint_ids(i)}.ub];
        phi = [phi;this_phi];
        J = [J; this_J];
        dJ = [dJ; this_dJ];
      end
      
      % todo: find a way to use Jdot*qd directly (ala Twan's code)
      % instead of computing dJ
      Jdotv = dJ*reshape(qd*qd',obj.num_positions^2,1);
      
      
      nf = numel(phi);
      lb_inds = phi<=lb; ub_inds = phi>=ub; % todo: add some tolerance here?
      Jlb = J(lb_inds,:);  Jub = J(phi>=ub,:); 
      Ain = [-Jlb*Hinv*J'; Jub*Hinv*J'];
      bin = [Jlb;-Jub]*Hinv*tau + [Jdotv(lb_inds);-Jdotv(ub_inds)];
      
      % todo: use fastqp first?
      prog = QuadraticProgram(eye(nf),zeros(nf,1),Ain,bin);
      f = prog.solve();
      vdot = Hinv*(tau + J'*f);
      
      % useful for debugging
      %if any(lb_inds)
      %  phiddot_lb = Jlb*Hinv*(tau+J'*f)+Jdotv(lb_inds)
      %end
      %if any(ub_inds)
      %  phiddot_ub = Jub*Hinv*(tau+J'*f)+Jdotv(ub_inds)
      %end
      
      xdot = [vToqdot(obj,q)*v; vdot];
    end    
  end
end
