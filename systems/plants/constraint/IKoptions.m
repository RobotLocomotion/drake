classdef IKoptions
% Q                  --the cost matrix for posture deviation
%                    -- use_mex, set to true if run with mex. default is
%                       false
% debug_mode         -- IK would query the name of each constraint
%
% The following fields is used in inverseKinPointwise
% sequentialSeedFlag -- A boolean variable, set to false if the seed of
%                       inverseKinPointwise is q_seed. If set to true,
%                       inverseKinPointwise would use the solution in the
%                       previous time point as seed (if it is feasible).
% All subsequent fields will be used in inverseKinTraj only
% Qa                 -- The cost matrix for acceleration
% Qv                 -- The cost matrix for velocity
% q0                 -- The initial posture
% qd0                -- The initial velocity
% qdf_ub             -- The upper bound of the final velocity
% qdf_lb             -- The lower bound of the final velocity
  properties(SetAccess=protected)
    robot
    nq
    Q
    Qa
    Qv
    use_mex
    debug_mode
    sequentialSeedFlag
    SNOPT_MajorFeasibilityTolerance
    SNOPT_MajorIterationsLimit
    SNOPT_IterationsLimit
    SNOPT_SuperbasicsLimit
    SNOPT_MajorOptimalityTolerance
    q0
    qd0
    qdf_ub
    qdf_lb
  end
  methods
    function obj = IKoptions(robot)
      obj.robot = robot;
      obj.nq = obj.robot.getNumDOF();
      obj.Q = eye(obj.nq);
      obj.Qa = 5*eye(obj.nq);
      obj.Qv = zeros(obj.nq);
%       obj.approximateFlag = false;
      obj.use_mex = true;
      obj.debug_mode = false;
      obj.sequentialSeedFlag = false;
      obj.SNOPT_MajorFeasibilityTolerance = 1e-6;
      obj.SNOPT_MajorIterationsLimit = 200;
      obj.SNOPT_IterationsLimit = 10000;
      obj.SNOPT_SuperbasicsLimit = 2000;
      obj.SNOPT_MajorOptimalityTolerance = 1e-4;
      obj.q0 = [];
      obj.qd0 = [];
      obj.qdf_ub = zeros(obj.nq,1);
      obj.qdf_lb = zeros(obj.nq,1);
    end
    
    function obj = setQ(obj,Q)
      typecheck(Q,'double');
      sizecheck(Q,[obj.nq,obj.nq]);
      Q = (Q+Q')/2;
      if(any(abs(eig(Q))<-1e-10))
        error('IKoptions:Q must be positive semidefinite');
      end
      obj.Q = Q;
    end
    
    function obj = setQa(obj,Qa)
      typecheck(Qa,'double');
      sizecheck(Qa,[obj.nq,obj.nq]);
      Qa = (Qa+Qa')/2;
      if(any(abs(eig(Qa))<-1e-10))
        error('IKoptions:Qa must be positive semidefinite');
      end
      obj.Qa = Qa;
    end
    
    function obj = setQv(obj,Qv)
      typecheck(Qv,'double');
      sizecheck(Qv,[obj.nq,obj.nq]);
      Qv = (Qv+Qv')/2;
      if(any(abs(eig(Qv))<-1e-10))
        error('IKoptions:Qv must be positive semidefinite');
      end
      obj.Qv = Qv;
    end
    
    function obj = setMex(obj,flag)
      sizecheck(flag,[1,1]);
      if(isa(flag,'logical'))
        obj.use_mex = flag;
      elseif(isa(flag,'double'))
        obj.use_mex = logical(flag);
      else
        error('IKoptions: Unsupported flag type');
      end
    end
    
    function obj = setDebug(obj,flag)
      sizecheck(flag,[1,1]);
      if(isa(flag,'logical'))
        obj.debug_mode = flag;
      elseif(isa(flag,'double'))
        obj.debug_mode = logical(flag);
      else
        error('IKoptions: Unsupported flag type');
      end
    end
    
    function obj = setSequentialSeedFlag(obj,flag)
      sizecheck(flag,[1,1]);
      if(isa(flag,'logical'))
        obj.sequentialSeedFlag = flag;
      elseif(isa(flag,'double'))
        obj.sequentialSeedFlag = logical(flag);
      else
        error('Drake:IKoptions: flag must be a logical scalar');
      end
    end
    
    function obj = setMajorOptimalityTolerance(obj,tol)
      sizecheck(tol,[1,1]);
      typecheck(tol,'double');
      if(tol<=0)
        error('Drake:IKoptions: tol must be positive');
      end
      obj.SNOPT_MajorOptimalityTolerance = tol;
    end
    
    function obj = setMajorFeasibilityTolerance(obj,tol)
      sizecheck(tol,[1,1]);
      typecheck(tol,'double');
      if(tol<=0)
        error('Drake:IKoptions: tol must be positive');
      end
      obj.SNOPT_MajorFeasibilityTolerance = tol;
    end
    
    function obj = setSuperbasicsLimit(obj,limit)
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_SuperbasicsLimit = floor(limit);
    end
    
    function obj = setMajorIterationsLimit(obj,limit)
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_MajorIterationsLimit = floor(limit);
    end
    
    function obj = setIterationsLimit(obj,limit)
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_IterationsLimit = floor(limit);
    end
    
    function obj = setq0(obj,q0)
      typecheck(q0,'double');
      sizecheck(q0,[obj.nq,1]);
      [joint_limit_min,joint_limit_max] = obj.robot.getJointLimits();
      q0 = min([q0 joint_limit_max],[],2);
      q0 = max([q0 joint_limit_min],[],2);
      obj.q0 = q0;
    end
    
    function obj = setqd0(obj,qd0)
      typecheck(qd0,'double');
      sizecheck(qd0,[obj.nq,1]);
      obj.qd0 = qd0;
    end
    
    
    function obj = setqdf(obj,lb,ub)
      typecheck(lb,'double');
      typecheck(ub,'double');
      sizecheck(lb,[obj.nq,1]);
      sizecheck(ub,[obj.nq,1]);
      obj.qdf_lb = lb;
      obj.qdf_ub = ub;
    end
  end
end
