classdef SearchContactsBase < ForceClosureContactsBase
  % This is the base class for searching the contact positions with some
  % lower bound on its Q1 metric. Derived from this base class, we will
  % handle the nonlinear friction cone and linearized friction cone
  % separately.
  properties(SetAccess = protected)
    % a'w+b>=0 is the halfspace that contains the wrench set
    a_indet  % The 6 x 1 indeterminates
    b_indet  % The 1 x 1 indeterminate
    
    disturbance_pos % A 3 x 1 vector. The position where the external disturbance wrench is applied
    % The wrench disturbance L2 ball is parameterized as w'*Qw*w<= r^2
    Qw    % A 6 x 6 PSD matrix.
    
    search_lagrangian_flag % True if lagrangian and contacts are search simultaneously in the BMI. Default is false
  end
  
  methods
    function obj = SearchContactsBase(A_xc, b_xc, disturbance_pos, num_contacts, mu_face, Qw, options)
      % options  A structure    -- robot   A RigidBodyManipulator object.
      %                                    default is empty
      %                         -- lin_fc_flag  a boolean, true if use
      %                            linearized friction cone, default is false
      %                         -- num_fc_edges An integer. The number of
      %                            edges used in the linearized friction
      %                            cone, default is 4
      if(nargin<7)
        options = struct();
      end
      if(~isfield(options,'robot'))
        options.robot = [];
      end
      if(~isfield(options,'lin_fc_flag'))
        options.lin_fc_flag = false;
      end
      if(~isfield(options,'num_fc_edges'))
        options.num_fc_edges = 4;
      end
      obj = obj@ForceClosureContactsBase(A_xc,b_xc,num_contacts,mu_face,options);
      if(any(size(disturbance_pos) ~= [3,1]))
        error('disturbance_pos should be a 3 x 1 vector');
      end
      obj.disturbance_pos = disturbance_pos;
      if(any(size(Qw) ~= [6,6]) || any(eig(Qw))<0)
        error('Qw should be a 6 x 6 PSD matrix');
      end
      obj.Qw = Qw;
      obj.search_lagrangian_flag = false;
      
      obj.a_indet = msspoly('a',6);
      obj.b_indet = msspoly('b',1);
      obj = obj.withIndeterminate(obj.a_indet);
      obj = obj.withIndeterminate(obj.b_indet);
      
    end
    
    function plotSolution(obj,sol,sol_bilinear)
      plotSolution@ForceClosureContactsBase(obj,sol,sol_bilinear);
      cone_length = det(obj.A_xc'*obj.A_xc)^(1/6);
      for i = 1:obj.num_contacts
        sol.fc{i}.plot(obj.use_lcmgl,cone_length,sprintf('fc%d',i));
      end
    end
  end
  
  methods(Access = protected)
    function obj = addXCC(obj)
      for i = 1:obj.num_contacts
        [obj,obj.XCC{i}] = obj.newSym(6);
        [obj,xcc_ind_i] = obj.addBilinearVariable([obj.xc(:,i);obj.c(:,i)],obj.XCC{i});
        obj.xc_ind(:,i) = xcc_ind_i(1:3);
        obj.c_ind(:,i) = xcc_ind_i(4:6);
      end
    end
  end
end