classdef FixedContactsSearchQ1Base < spotsosprog
  % This is the base class for searching over Lagrangian multiplier to
  % maximize the Q1 metric, given fixed contact positions and friction
  % cones. From this base class we will derive the class for linearized
  % friction cone and nonlinear friction cone
  properties(SetAccess = protected)
    % The halfspace is parameterized as a_indet'*w+b_indet>=0
    a_indet % A 6 x 1 vector
    b_indet % A 1 x 1 scalar
    
    disturbance_pos  % A 3 x 1 vector. The position where the disturbance wrench is applied. The Q1 metric is measured about the norm of the disturbance wrench applied at this position
    num_contacts  % The number of contact points
    Qw   % A 6 x 6 PSD matrix, w'*Qw*w is the norm in the wrench space
  end
  
  properties
    backoff_flag   % True if we do a backoff stage after optimizing with objective functions. Default is true
    backoff_scale  % This is used in the "backoff" stage of optimization, default value is 0.98
  end
  
  methods
    function obj = FixedContactsSearchQ1Base(disturbance_pos,num_contacts,Qw)
      obj = obj@spotsosprog();
      obj.a_indet = msspoly('a',6);
      obj.b_indet = msspoly('b',1);
      obj = obj.withIndeterminate(obj.a_indet);
      obj = obj.withIndeterminate(obj.b_indet);
      if(any(size(disturbance_pos)~=[3,1]))
        error('disturbance_pos should be a 3 x 1 vector');
      end
      obj.disturbance_pos = disturbance_pos;
      if(numel(num_contacts) ~= 1 || num_contacts<1)
        error('num_contacts should be a positive integer');
      end
      obj.num_contacts = num_contacts;
      if(any(size(Qw)~=[6,6]) || any(eig(Qw))<0)
        error('Qw should be a 6 x 6 PSD matrix');
      end
      obj.Qw = Qw;
      obj.backoff_flag = true;
      obj.backoff_scale = 0.95;
    end
  end
end