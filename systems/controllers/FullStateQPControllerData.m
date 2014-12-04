classdef FullStateQPControllerData < ControllerData
  % Class that contains data needed by the FullStateQPController
  
  % properties that change infrequently or never
  properties (SetAccess=private,GetAccess=public)
    A % state map
    A_is_time_varying=false 
    B % input map
    B_is_time_varying=false 
  end
  
  % properties that can be modified 'on the fly'
  properties (SetAccess=public,GetAccess=public)
    % solver related -------------------------------------------------------------
    infocount=0 % number of consecutive iterations with solver info < 0
    qp_active_set=[]% active set of inequality constraints from pervious iteration
    
    % LQR solution terms ---------------------------------------------------------
    x0 % nominal state (possibly time-varying)
    u0 % nominal input (possibly time-varying)
    R % input LQR cost
    S % cost-to-go terms: x'Sx + x's1 + s2
    s1
    s2
    lqr_is_time_varying % true if TVLQR, false if TILQR
    link_constraints = []
    ignore_terrain = false
 
    % Contact stuff --------------------------------------------------------------
    support_times % vector of contact transition times
    supports % (array of) RigidBodySupportState object(s)
  end
  
  methods 
    function obj = FullStateQPControllerData(lqr_is_time_varying,data)
      typecheck(lqr_is_time_varying,'logical');
      typecheck(data,'struct');
      data.lqr_is_time_varying = lqr_is_time_varying; % true if LQR problem is tv
      obj = obj@ControllerData(data);
    end
 
    function data=verifyControllerData(~,data)
      if data.lqr_is_time_varying
        assert(isa(data.x0,'Trajectory'));
        assert(isa(data.u0,'Trajectory'));
        assert(isa(data.S,'Trajectory') || isa(data.S,'cell')); 
        if isfield(data,'s1')
          assert(isa(data.s1,'Trajectory') || isa(data.s1,'cell'));
        end
        if isfield(data,'s2')
          assert(isa(data.s2,'Trajectory') || isa(data.s2,'cell'));
        end
      else
        assert(isnumeric(data.x0));
        assert(isnumeric(data.u0));
        assert(isnumeric(data.S));
      end
      assert(isnumeric(data.support_times));
      assert(isnumeric(data.R));
      if isfield(data,'A')
        assert(isnumeric(data.A) || isa(data.A,'Trajectory') || isa(data.A,'cell'));
      end
      if isfield(data,'B')
        assert(isnumeric(data.B) || isa(data.B,'Trajectory') || isa(data.B,'cell'));
      end
    end
        
    function updateControllerData(obj,data)
      if isfield(data,'lqr_is_time_varying')
        obj.lqr_is_time_varying = data.lqr_is_time_varying;
      end
      if isfield(data,'S')
        obj.S = data.S;
      end
      if isfield(data,'s1')
        obj.s1 = data.s1;
      end
      if isfield(data,'s2')
        obj.s2 = data.s2;
      end
      if isfield(data,'x0')
        obj.x0 = data.x0;
      end
      if isfield(data,'u0')
        obj.u0 = data.u0;
      end
      if isfield(data,'supports')
        obj.supports = data.supports;
      end
      if isfield(data,'support_times')
        obj.support_times = data.support_times;
      end
      if isfield(data,'qp_active_set')
        obj.qp_active_set = data.qp_active_set;
      end
      if isfield(data,'R')
        obj.R = data.R;
      end
      if isfield(data,'A') 
        if isa(data.A,'Trajectory') || isa(data.A,'cell')
          obj.A_is_time_varying = true;
        end
        obj.A = data.A;
      end
      if isfield(data,'B')
        if isa(data.B,'Trajectory') || isa(data.B,'cell')
          obj.B_is_time_varying = true;
        end
        obj.B = data.B;
      end
    end
  end
end