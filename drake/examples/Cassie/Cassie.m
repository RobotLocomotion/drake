classdef Cassie < TimeSteppingRigidBodyManipulator 
  methods

    function obj=Cassie(urdf,options)
      if nargin < 1
        urdf = fullfile(getDrakePath(), 'examples', 'Cassie', 'urdf', 'half_cassie_minimal.urdf');
      else
        typecheck(urdf,'char');
      end
      if nargin < 2
        options = struct();
      end
      if ~isfield(options,'dt')
        options.dt = 0.0005;
      end
      if ~isfield(options,'floating')
        options.floating = true;
      end
      if ~isfield(options,'terrain')
        options.terrain = RigidBodyFlatTerrain;
      end
      
      w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');

      obj = obj@TimeSteppingRigidBodyManipulator(urdf,options.dt,options);
      warning(w);

      obj.left_foot_id =  obj.findLinkId('toe_left');
      obj.right_foot_id =  obj.findLinkId('toe_right');

      try
          THIGH_LEFT_ID = obj.findLinkId('thigh_left');
          THIGH_RIGHT_ID = obj.findLinkId('thigh_right');
          HEEL_SPRING_LEFT_ID = obj.findLinkId('heel_spring_left');
          HEEL_SPRING_RIGHT_ID = obj.findLinkId('heel_spring_right');


          rel_dist_left = @(x) obj.relDistQ(x,THIGH_LEFT_ID,[0;0;0.0045],HEEL_SPRING_LEFT_ID,[0.11877; -0.0001; 0],0.5012);
          rel_dist_right = @(x) obj.relDistQ(x,THIGH_RIGHT_ID,[0;0;0.0045],HEEL_SPRING_RIGHT_ID,[0.11877; -0.0001; 0],0.5012);

          con = FunctionHandleConstraint(0,0,obj.getNumPositions,rel_dist_left);
          con = con.setName('left-achilles-rod');
          obj.manip = addPositionEqualityConstraint(obj.manip,con); % addStateConstraint(obj,con);

          con = FunctionHandleConstraint(0,0,obj.getNumPositions,rel_dist_right);
          con = con.setName('right-achilles-rod');
          obj.manip = addPositionEqualityConstraint(obj.manip,con); % addStateConstraint(obj,con);
      catch
          disp('Cassie: skipping 4-bar constraints'); 
      end
      obj = compile(obj);
      
    end
    
    function [x,success,prog] = resolveConstraintsZeroSprings(obj,varargin)

      % add constraints for spring deflections 
      idx = 4+[obj.findJointId('knee_shin_passive_left');
          obj.findJointId('knee_shin_passive_right');
          obj.findJointId('heel_spring_joint_left');
          obj.findJointId('heel_spring_joint_right')];
 
      con = BoundingBoxConstraint(zeros(4,1),zeros(4,1));
      con = con.setName('springs');
      obj = addStateConstraint(obj,con,idx);
          
      [x,success,prog] = obj.resolveConstraints(varargin{:});
    end
        
    function [f,dfdx] = relDist(obj,x,b1,v1,b2,v2,d)
      kinsol = obj.doKinematics(x);
      [p1,J1] = obj.forwardKin(kinsol,b1,v1);
      [p2,J2] = obj.forwardKin(kinsol,b2,v2);
      f = norm(p1-p2) - d;
      dfdq = (p1-p2)'./norm(p1-p2) * (J1-J2);
      dfdx = [dfdq, 0*dfdq];
    end

    function [f,dfdx] = relDistQ(obj,x,b1,v1,b2,v2,d)
      kinsol = obj.doKinematics(x);
      [p1,J1] = obj.forwardKin(kinsol,b1,v1);
      [p2,J2] = obj.forwardKin(kinsol,b2,v2);
      f = norm(p1-p2) - d;
      dfdx = (p1-p2)'./norm(p1-p2) * (J1-J2);
    end
    
    function obj = compile(obj)
      obj = compile@TimeSteppingRigidBodyManipulator(obj);

      % Construct state vector itself -- start by replacing the
      % atlasPosition and atlasVelocity frames with a single
      % larger state frame
      if (strcmp(obj.manip.getStateFrame().getFrameByNum(1).name, 'cassiePosition'))
        cassie_state_frame = cassieFrames.CassieState(obj);
      else
        cassie_state_frame = obj.manip.getStateFrame();
        cassie_state_frame = replaceFrameNum(cassie_state_frame,1,cassieFrames.CassieState(obj));
      end

      obj.manip = obj.manip.setStateFrame(cassie_state_frame);
      obj = obj.setStateFrame(cassie_state_frame);

      obj.manip = obj.manip.setOutputFrame(cassie_state_frame);
      obj = obj.setOutputFrame(cassie_state_frame);

      input_frame = cassieFrames.CassieInput(obj);
 
      obj = obj.setInputFrame(input_frame);
      obj.manip = obj.manip.setInputFrame(input_frame);

    end   
    
  end

  
  
  properties
    left_foot_id
    right_foot_id
      
  end
end
