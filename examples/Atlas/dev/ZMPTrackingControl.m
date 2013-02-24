classdef ZMPTrackingControl < DrakeSystem
% input: atlas robot state
% state: com, comdot
% output: q_d
  
  methods 
    function obj = ZMPTrackingControl(r,x0)
      
%      obj = obj@DrakeSystem(2,0,r.getNumStates(),r.getNumInputs,false,true);
      obj = obj@DrakeSystem(0,2+getNumDOF(r),r.getNumStates(),r.getNumInputs,false,true);
      obj = setSampleTime(obj,[.005;0]); % sets controller update rate

      obj.robot = r;
      
      % nominal position goal
      q0 = x0(1:getNumDOF(r));

      kinsol = doKinematics(r,q0);
      rfoot_body = r.findLink('r_foot');
      lfoot_body = r.findLink('l_foot');

      rfoot0 = forwardKin(r,kinsol,rfoot_body,[0;0;0]);
      lfoot0 = forwardKin(r,kinsol,lfoot_body,[0;0;0]);

      gc = r.contactPositions(q0);

      % compute desired COM projection
      % assumes minimal contact model for now
      k = convhull(gc(1:2,1:4)');
      lfootcen = [mean(gc(1:2,k),2);0];
      k = convhull(gc(1:2,5:8)');
      rfootcen = [mean(gc(1:2,4+k),2);0];

      midfoot = mean([rfootcen,lfootcen],2);
      com0 = getCOM(r,kinsol);

      % create trajectory
      com = [com0,midfoot,rfootcen,rfootcen,rfootcen,rfootcen,rfootcen,midfoot,lfootcen,lfootcen,lfootcen,lfootcen,lfootcen,midfoot];
      com(3,:) = com0(3);
      tstep = .5*((1:size(com,2))-1);

      ts = linspace(0,tstep(end),100);
      cost = Point(r.getStateFrame,1);
      cost.pelvis_x = 0;
      cost.pelvis_y = 0;
      cost.pelvis_z = 0;
      cost.pelvis_roll = 1000;
      cost.pelvis_pitch = 1000;
      cost.pelvis_yaw = 0;
      cost.back_mby = 100;
      cost.back_ubx = 100;
      cost = double(cost);
      obj.ikoptions.Q = diag(cost(1:r.getNumDOF));
      obj.ikoptions.q_nom = q0;

      addpath(fullfile(getDrakePath,'examples','ZMP'));
      obj.comgoal = PPTrajectory(foh(tstep,com));
      limp = LinearInvertedPendulum2D(com(3,1));
      zmp = setOutputFrame(obj.comgoal(2),desiredZMP1D);
      obj.tvcontrol = ZMPtracker(limp,zmp);
      obj.tvcontrol = inOutputFrame(inInputFrame(obj.tvcontrol,getStateFrame(limp)),getInputFrame(limp));
      
      obj.rfoot_body = r.findLink('r_foot');
      obj.lfoot_body = r.findLink('l_foot');

      rfootpos = repmat([rfoot0;0;0;0],1,length(tstep));
      lfootpos = repmat([lfoot0;0;0;0],1,length(tstep));
      lfootpos(3,5) = .15;
      rfootpos(3,11) = .15;

      obj.rfootpos = PPTrajectory(foh(tstep,rfootpos));
      obj.lfootpos = PPTrajectory(foh(tstep,lfootpos));
      obj.q0 = q0;
    end
    
    function state = getInitialStateWInput(obj,t,state,x)
      nQ = getNumDOF(obj.robot);
      q = x(1:nQ); qd = x(nQ+(1:nQ));
      
      [com,Jcom] = getCOM(obj.robot,obj.q0);
      comdot = Jcom*qd;
      state = [com(2);comdot(2);obj.q0];
    end
    
    function comstatedot = dynamics(obj,t,comstate,x)
      nQ = getNumDOF(obj.robot);
      q = x(1:nQ); qd = x(nQ+(1:nQ));
      
%      [com,Jcom] = getCOM(obj.robot,q);
%      comdot = Jcom*qd;
%      comddot_d = obj.tvcontrol.output(t,[],[com(2);comdot(2)])
%      comstatedot = [comdot(2);comddot_d];

      comddot_d = obj.tvcontrol.output(t,[],comstate);
      comstatedot = [comstate(2);comddot_d];
    end
    
    function next_state = update(obj,t,state,x)
      comstate=state(1:2);
      comstatedot = dynamics(obj,t,comstate,x);
      ts = getSampleTime(obj);
      
%      nQ = getNumDOF(obj.robot);
%      q = x(1:nQ);
      q = state(3:end);
      
      scope('Atlas','y_com_d',t,comstate(1));

      com_d = obj.comgoal.eval(t);
      com_d(2) = comstate(1);
      q_ik = inverseKin(obj.robot,q,0,com_d,obj.rfoot_body,[0;0;0],obj.rfootpos.eval(t),obj.lfoot_body,[0;0;0],obj.lfootpos.eval(t),obj.ikoptions);
      next_state = [comstate + ts(1)*comstatedot; q_ik];
    end
    
    function qa_d = output(obj,t,state,x)
      q = state(3:end);
      qa_d = q(getActuatedJoints(obj.robot));
    end
    
    function tspan = getTspan(obj)
      tspan = obj.rfootpos.tspan;
    end
    
  end
  
  properties
    robot
    tvcontrol
    q0
    ikoptions
    rfoot_body
    lfoot_body
    rfootpos
    lfootpos
    comgoal
  end
  
end