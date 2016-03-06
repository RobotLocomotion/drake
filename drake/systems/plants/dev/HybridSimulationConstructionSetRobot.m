classdef HybridSimulationConstructionSetRobot < HybridDrakeSystem
% Simulation Construction Set robots that have ground contact models or
% collision models are not smooth.  Use this class to wrap them in a hybrid
% model. 
  
  methods
    function obj=HybridSimulationConstructionSetRobot(robotobj)
      obj = obj@HybridDrakeSystem();
      typecheck(robotobj,'com.yobotics.simulationconstructionset.Robot');

      obj.robotobj = robotobj;  %store a local copy, too
      obj.gc_model = robotobj.getGroundContactModel();
      obj.num_gc_points = robotobj.getGroundContactPoints().size();
      
      % first attempt: create 2^num_gc_points modes (ouch!)
      for i=1:2^obj.num_gc_points
        obj = addMode(obj,SimulationConstructionSetRobot(robotobj));
      end
      for i=1:2^obj.num_gc_points
        gcs=GCbooleanFromModeNum(obj,i);
        for j=1:obj.num_gc_points % consider each GC flipping
          gcs(j)= not(gcs(j));
          to_mode=modeNumFromGCboolean(obj,gcs);
          obj = addTransition(obj,i,@(obj,t,x,u)groundContactGuard(obj,t,x,u,i,j),@(obj,m,t,x,u)groundContactTransition(obj,m,t,x,u,to_mode),false,false,to_mode);
          gcs(j)= not(gcs(j));
        end
      end
      obj = setModeOutputFlag(obj,false);
    end
    
    function modenum=modeNumFromGCboolean(obj,gcs)
      gcs = logical(gcs);
      sizecheck(gcs,[obj.num_gc_points,1]);
      modenum = bin2dec(sprintf('%d',gcs))+1;
    end
    function gcs=GCbooleanFromModeNum(obj,modenum)
      gcs = logical(sscanf(dec2bin(modenum-1),'%1d'));
      gcs = [repmat(false,obj.num_gc_points-length(gcs),1);gcs];
    end
      
    function phi = groundContactGuard(obj,t,mode_x,u,from_mode,gc_that_flips_ind)
      phi = zeroCrossings(obj,t,[from_mode;mode_x],u);
      % just extract the one that I care about.
      phi = phi(gc_that_flips_ind);
    end

    function zcs = zeroCrossings(obj,t,x,u)
      % overload loop through guards since I can (and sort of have to) make 
      % a single call to evaluate them all
      m = x(1);
      xm = x(1+(1:getNumStates(obj.modes{m})));
      gcs = GCbooleanFromModeNum(obj,m);
      % returns signed distance to ground for each ground contact point
      zcs = obj.robotobj.matlabGroundContactGuards(xm);
      
      % flip the sign of the ones that are already in the ground (want to
      % trigger them when they leave the ground)
      zcs(gcs)=-zcs(gcs);
    end

    function [to_mode_xn,to_mode_num,status] = groundContactTransition(obj,from_mode,t,mode_x,u,to_mode_num)
      to_mode_xn = obj.robotobj.matlabGroundContactTransition(mode_x);
      status=0;
    end
    
    
  end
  
  properties
    robotobj;
    gc_model;
    num_gc_points;
  end
  
end
