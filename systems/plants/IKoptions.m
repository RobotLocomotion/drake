classdef IKoptions
% @param Q                  --the cost matrix for posture deviation
% @param use_mex,           -- A boolean flat,set to true if run with mex. default is
%                              false
% @param debug_mode         -- IK would query the name of each constraint
%
% The following fields is used in inverseKinPointwise
% @param sequentialSeedFlag -- A boolean variable, set to false if the seed of
%                             inverseKinPointwise is q_seed. If set to true,
%                             inverseKinPointwise would use the solution in the
%                             previous time point as seed (if it is feasible).
%
% All subsequent fields will be used in inverseKinTraj only
% @param Qa                 -- The cost matrix for acceleration
% @param Qv                 -- The cost matrix for velocity
% @param additional_tSamples  -- Additional time samples (apart from knot points) in inverseKinTraj to check
%                                constraints
% @param fixInitialState    -- A boolean flag, set to true if the posture q and
%                              velocity qd at time t(1) is fixed. q(1) = q_seed_traj.eval(t(1))
%                              qdot(1) = (ikpotions.qd0_lb+ikoptions.qd0_ub)/2
% @param q0_lb              -- The initial posture upperbound. Would be used when
%                              fixInitialState = false;
% @param q0_ub              -- The initial posture lowerbound, Would be used when
%                              fixInitialState = false;
% @param qd0_ub             -- The upperbound of the initial velocity
% @param qd0_lb             -- The lowerbound of the initial velocity
% @param qdf_ub             -- The upper bound of the final velocity
% @param qdf_lb             -- The lower bound of the final velocity
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
    additional_tSamples
    fixInitialState
    q0_lb
    q0_ub
    qd0_lb
    qd0_ub
    qdf_ub
    qdf_lb
    mex_ptr = nullPointer();
  end

  methods(Access = protected)
    function obj = setDefaultParams(obj,robot)
      obj.robot = robot;
      obj.nq = obj.robot.getNumPositions();
      obj.Q = eye(obj.nq);
      obj.Qa = 0.1*eye(obj.nq);
      obj.Qv = zeros(obj.nq);
      obj.use_mex = logical(exist('inverseKinmex','file'));
      obj.debug_mode = true;
      obj.sequentialSeedFlag = false;
      obj.SNOPT_MajorFeasibilityTolerance = 1e-6;
      obj.SNOPT_MajorIterationsLimit = 200;
      obj.SNOPT_IterationsLimit = 10000;
      obj.SNOPT_SuperbasicsLimit = 2000;
      obj.SNOPT_MajorOptimalityTolerance = 1e-4;
      obj.additional_tSamples = [];
      obj.fixInitialState = true;
      [obj.q0_lb,obj.q0_ub] = obj.robot.getJointLimits();
      obj.qd0_ub = zeros(obj.nq,1);
      obj.qd0_lb = zeros(obj.nq,1);
      obj.qdf_ub = zeros(obj.nq,1);
      obj.qdf_lb = zeros(obj.nq,1);
    end
  end

  methods
    function obj = IKoptions(robot)
      obj = setDefaultParams(obj,robot);
      if robot.getMexModelPtr~=0 && exist('IKoptionsmex','file')
        ptr = IKoptionsmex(1,robot.getMexModelPtr);
        obj.mex_ptr = ptr;
      end
    end

    function obj = setQ(obj,Q)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'Q',Q);
      end
      typecheck(Q,'double');
      sizecheck(Q,[obj.nq,obj.nq]);
      Q = (Q+Q')/2;
      if(any(abs(eig(Q))<0))
        error('IKoptions:Q must be positive semidefinite');
      end
      obj.Q = Q;
    end

    function obj = setQa(obj,Qa)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'Qa',Qa);
      end
      typecheck(Qa,'double');
      sizecheck(Qa,[obj.nq,obj.nq]);
      Qa = (Qa+Qa')/2;
      if(any(abs(eig(Qa))<0))
        error('IKoptions:Qa must be positive semidefinite');
      end
      obj.Qa = Qa;
    end

    function obj = setQv(obj,Qv)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'Qv',Qv);
      end
      typecheck(Qv,'double');
      sizecheck(Qv,[obj.nq,obj.nq]);
      Qv = (Qv+Qv')/2;
      if(any(abs(eig(Qv))<0))
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
      if obj.use_mex && obj.mex_ptr==0
        warning('Drake:IKoptions:MexDisabled','You asked to enable mex, but mex support is currently disabled (because the IK mex has not been built)');
        obj.use_mex = false;
      end
    end

    function obj = setDebug(obj,flag)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'debug',flag);
      end
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
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'sequentialSeedFlag',flag);
      end
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
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'majorOptimalityTolerance',tol);
      end
      sizecheck(tol,[1,1]);
      typecheck(tol,'double');
      if(tol<=0)
        error('Drake:IKoptions: tol must be positive');
      end
      obj.SNOPT_MajorOptimalityTolerance = tol;
    end

    function obj = setMajorFeasibilityTolerance(obj,tol)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'majorFeasibilityTolerance',tol);
      end
      sizecheck(tol,[1,1]);
      typecheck(tol,'double');
      if(tol<=0)
        error('Drake:IKoptions: tol must be positive');
      end
      obj.SNOPT_MajorFeasibilityTolerance = tol;
    end

    function obj = setSuperbasicsLimit(obj,limit)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'superbasicsLimit',limit);
      end
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_SuperbasicsLimit = floor(limit);
    end

    function obj = setMajorIterationsLimit(obj,limit)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'majorIterationsLimit',limit);
      end
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_MajorIterationsLimit = floor(limit);
    end

    function obj = setIterationsLimit(obj,limit)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'iterationsLimit',limit);
      end
      sizecheck(limit,[1,1]);
      typecheck(limit,'double');
      if(limit<1)
        error('Drake:IKoptions: limit must be positive integers');
      end
      obj.SNOPT_IterationsLimit = floor(limit);
    end

    function obj = setFixInitialState(obj,flag)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'fixInitialState',flag);
      end
      sizecheck(flag,[1,1]);
      if(isa(flag,'logical'))
        obj.fixInitialState = flag;
      elseif(isa(flag,'double'))
        obj.fixInitialState = logical(flag);
      else
        error('IKoptions: Unsupported flag type');
      end
    end

    function obj = setq0(obj,lb,ub)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'q0',lb,ub);
      end
      typecheck(lb,'double');
      typecheck(ub,'double');
      sizecheck(lb,[obj.nq,1]);
      sizecheck(ub,[obj.nq,1]);
      if(any(lb>ub))
        error('IKoptions:setq0: lb must be no larger than ub');
      end
      [lb_tmp,ub_tmp] = obj.robot.getJointLimits();
      obj.q0_lb = max([lb lb_tmp],[],2);
      obj.q0_ub = min([ub ub_tmp],[],2);
    end

    function obj = setqd0(obj,lb,ub)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'qd0',lb,ub);
      end
      typecheck(lb,'double');
      typecheck(ub,'double');
      sizecheck(lb,[obj.nq,1]);
      sizecheck(ub,[obj.nq,1]);
      if(any(lb>ub))
        error('IKoptions:setqd0: lb must be no larger than ub');
      end
      obj.qd0_lb = lb;
      obj.qd0_ub = ub;
    end


    function obj = setqdf(obj,lb,ub)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'qdf',lb,ub);
      end
      typecheck(lb,'double');
      typecheck(ub,'double');
      sizecheck(lb,[obj.nq,1]);
      sizecheck(ub,[obj.nq,1]);
      if(any(lb>ub))
        error('IKoptions:setqdf: lb must be no larger than ub');
      end
      obj.qdf_lb = lb;
      obj.qdf_ub = ub;
    end

    function obj = setAdditionaltSamples(obj,t_samples)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'additionaltSamples',t_samples);
      end
      typecheck(t_samples,'double');
      if(~isempty(t_samples))
        sizecheck(t_samples,[1,nan]);
        obj.additional_tSamples = unique(t_samples);
      else
        obj.additional_tSamples = [];
      end
    end

    function obj = updateRobot(obj,robot)
      if obj.mex_ptr ~= 0
        obj.mex_ptr = IKoptionsmex(3,obj.mex_ptr,'robot',robot.getMexModelPtr);
      end
      obj.robot = robot;
      nq_cache = obj.nq;
      obj.nq = obj.robot.getNumPositions();
      if(obj.nq ~= nq_cache)
        obj = setDefaultParams(obj,robot);
      end
    end
  end
end
