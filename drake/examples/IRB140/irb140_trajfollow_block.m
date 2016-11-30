classdef irb140_trajfollow_block < MIMODrakeSystem
  
  properties
    % Atlas and various controllers:
    r;
    xtraj;
    nq;
    lcmgl; 
  end
  
  methods
    function obj = irb140_trajfollow_block(r, xtraj, input_override, options)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      if nargin<3
        input_override = r.getOutputFrame();
      end
      if nargin<4
        options = struct();
      end

      % Generate AtlasInput as out (we'll do translation manually)
      output_frame = getInputFrame(r);
      
      % We'll need state as input
      input_frame = input_override;
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,false);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      
      obj.r = r;
      obj.xtraj = xtraj;
      obj.nq = r.getNumPositions;

      checkDependency('lcmgl');
      obj.lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'irb140 target pose');
    
    end
    
    function varargout=mimoOutput(obj,t,~,state)
      % proof-of-concept end-effector control. Will presumably be
      % replaced with something fancier and better-performing
      K_P = 10000;
      K_D = 1000;
      K_D_P = 100;
      target_pose = obj.xtraj.eval(t);
      error = target_pose - state;
      efforts = K_P*error(1:obj.nq) + K_D*error(1:obj.nq) + K_D_P*error(obj.nq+1:end);
      varargout = {efforts};
      
      % Draw target end-effector position (or something in line with it,
      % anyway, since true end-effector position isn't yet figured out
      % for the real arm) for reference.
      if ~isempty(obj.lcmgl)
        obj.lcmgl.glColor3f(1, 0, 0);
        kinsol = doKinematics(obj.r.getManipulator,target_pose(1:obj.r.getNumPositions));
        pt = forwardKin(obj.r.getManipulator,kinsol,6,[0.3,0,0.0].');
        obj.lcmgl.glTranslated(pt(1), pt(2), pt(3));
        obj.lcmgl.box([0,0,0], [0.025, 0.025, 0.025]);
        obj.lcmgl.switchBuffers;
      end
      
    end
  end
  
end
