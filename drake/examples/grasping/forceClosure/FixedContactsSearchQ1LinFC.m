classdef FixedContactsSearchQ1LinFC < FixedContactsSearchQ1Base
  % For fixed contacts and friction cones, find the Q1 metric using SOS
  % conditions.
  properties(SetAccess = protected)
    monomial0 % The monomial vector of the Lagrangian multiplier, for condition b-r>=0
    monomial1 % The monomial vector of the Lagrangian multiplier, for constraint 1 - a'*inv(Qw)*a = 0
    monomial2 % The monomial vector of the Lagrangian multiplier, for constraint a^T*Vij+b >=0
    
    L1_mat % The monomial matrix of the Lagrangian multiplier, for constraint 1 - a'*inv(Qw)*a = 0
    L2_mat % A num_contacts x num_fc_edges cell, L1_mat{i,j} is the monomial matrix of the Lagrangian multiplier, for constraint a^T*V_ij+b>=0
    
    radius % The Q1 metric
    num_fc_edges % The number of edges in each linearized friction cone
  end
  
  methods
    function obj = FixedContactsSearchQ1LinFC(disturbance_pos,num_contacts,num_fc_edges,Qw)
      obj = obj@FixedContactsSearchQ1Base(disturbance_pos,num_contacts,Qw);
      if(numel(num_fc_edges) ~= 1 || num_fc_edges<1)
        error('num_fc_edges should be a positive integer');
      end
      obj.num_fc_edges = num_fc_edges;
      [obj,obj.radius] = obj.newFree(1,1);
      obj = obj.withPos(obj.radius);
      
      obj.monomial0 = monomials([obj.a_indet;obj.b_indet],0:1);
      obj.monomial1 = monomials([obj.a_indet;obj.b_indet],0:1);
      obj.monomial2 = monomials([obj.a_indet;obj.b_indet],0:1);
      obj.L2_mat = cell(obj.num_contacts,obj.num_fc_edges);
      [obj, obj.L1_mat] = obj.newSym(length(obj.monomial1));
      for i = 1:obj.num_contacts
        for j = 1:obj.num_fc_edges
          [obj, obj.L2_mat{i,j}] = obj.newSym(length(obj.monomial2));
          obj = obj.withPSD(obj.L2_mat{i,j});
        end
      end
      [obj,obj.radius] = obj.newFree(1,1);
      obj = obj.withPos(obj.radius);
    end
    
    function [solver_sol,info,solver_time] = searchQ1(obj,m_L0_mat,friction_cones)
      % @param friction_cones, a cell of LinearizedFrictionCone objects
      % @param m_L0_mat, the monomial matrix of the Lagrangian multiplier,
      % for condition b >= r
      V = obj.sosQ1condition(m_L0_mat,friction_cones,obj.radius);
      obj = obj.withSOS(V);
      
      options = spot_sdp_default_options();
      options.verbose = 0;
      solver_sol = obj.minimize(-obj.radius,@spot_mosek,options);
      solver_time = solver_sol.info.wtime;
      if(solver_sol.isPrimalFeasible())
        if(obj.backoff_flag)
          radius_sol = double(solver_sol.eval(obj.radius));
          radius_lb = obj.backoff_scale*radius_sol;
%           radius_lb = max(radius_lb,radius_min);
          obj = obj.withPos(obj.radius - radius_lb);
          solver_sol = obj.minimize(0,@spot_mosek,options);
          if(~solver_sol.isPrimalFeasible())
            error('The program fails in backoff stage');
          end
        end
        info = 1;
      else
        info = 0;
      end
    end
    
    function m_L0_mat = findL0Mat(obj,friction_cones)
      % Find a m_L0_mat such that it can verify Q1 metric is larger than 0
      % @param friction_cones, a cell of LinearizedFrictionCone objects
      [obj,L0_mat] = obj.newSym(length(obj.monomial0));
      obj = obj.withSOS(obj.monomial0'*L0_mat*obj.monomial0 - 1e-4);
      V = obj.sosQ1condition(L0_mat,friction_cones,0);
      obj = obj.withSOS(V);
      
      options = spot_sdp_default_options();
      options.verbose = 0;
      solver_sol = obj.minimize(0,@spot_mosek,options);
      if(solver_sol.isPrimalFeasible())
        m_L0_mat = double(solver_sol.eval(L0_mat));
      else
        error('Cannot verify 0 is a lower bound of the Q1 metric through SOS condition');
      end
    end
    
    function sol = retrieveSolution(obj,solver_sol)
      sol.radius = double(solver_sol.eval(obj.radius));
      sol.L2_mat = cell(obj.num_contacts,obj.num_fc_edges);
      sol.L1_mat = double(solver_sol.eval(obj.L1_mat));
      for i = 1:obj.num_contacts
        for j = 1:obj.num_fc_edges
          sol.L2_mat{i,j} = double(solver_sol.eval(obj.L2_mat{i,j}));
        end
      end
    end
  end
  
  methods(Access = protected)
    function V = sosQ1condition(obj,m_L0_mat,friction_cones,radius)
      if(~iscell(friction_cones) || length(friction_cones) ~= obj.num_contacts);
        error('friction_cones should be a %d x 1 cell',obj.num_contacts);
      end
      V = (obj.monomial0'*m_L0_mat*obj.monomial0)*(obj.b_indet - radius) - obj.monomial1'*obj.L1_mat*obj.monomial1*(1 - obj.a_indet'*inv(obj.Qw)*obj.a_indet);
      contact_pos = zeros(3,obj.num_contacts);
      for i = 1:obj.num_contacts;
        contact_pos(:,i) = friction_cones{i}.contact_pos;
      end
      G = graspTransferMatrix(contact_pos - bsxfun(@times,obj.disturbance_pos,ones(1,obj.num_contacts))); 
      for i = 1:obj.num_contacts
        Vi = G(:,(i-1)*3+(1:3))*friction_cones{i}.fc_edges;
        for j = 1:obj.num_fc_edges
          V = V - obj.monomial2'*obj.L2_mat{i,j}*obj.monomial2*(Vi(:,j)'*obj.a_indet + obj.b_indet);
        end
      end
    end
  end
end