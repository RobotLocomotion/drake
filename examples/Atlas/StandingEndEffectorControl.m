classdef StandingEndEffectorControl < MIMODrakeSystem
% A simple jacobian-based standing controller that regulates the center of 
% mass while achieving end effector goals in the nullspace 
  methods
    function obj = StandingEndEffectorControl(sys,r,k_com,k_nom)
      % @param sys the PD-controlled closed-loop system
      % @param r the time-stepping manipulator
      % @param k_com (optional) the COM objective gain      
      % @param k_nom (optional) the nominal configuration gain      
      
      typecheck(sys,'DrakeSystem');
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      
      comframe = AtlasCOM(r);
      qnomframe = AtlasJointConfig(r,true);
      support_frame = AtlasBody(r);
    
      input_frame = MultiCoordinateFrame({support_frame,comframe,qnomframe,sys.getStateFrame});
      output_frame = AtlasPositionRef(r);

      obj = obj@MIMODrakeSystem(0,sys.getInputFrame.dim,input_frame,output_frame,false,true);
      obj = setSampleTime(obj,[.01;0]); % sets controller update rate
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
      obj.manip = r;
      
      if nargin > 2
        typecheck(k_com,'double');
        obj.k_com = k_com;
      end
      if nargin > 3
        typecheck(k_nom,'double');
        obj.k_nom = k_nom;
      end

      x0 = r.getInitialState();
      B = r.getB();
      obj = setInitialState(obj,B' * x0(1:r.getNumDOF()));
      
      % set max commanded positions to joint limits
      [obj.q_d_min,obj.q_d_max] = obj.manip.getJointLimits();
      obj.q_d_max(obj.q_d_max == inf) = 1e10;
      obj.q_d_min(obj.q_d_min == -inf) = -1e10;
      obj.q_d_max = B' * obj.q_d_max;
      obj.q_d_min = B' * obj.q_d_min;
      
    end   
    
    function obj = addEndEffector(obj,end_effector)
      typecheck(end_effector,'EndEffector');
      obj.end_effectors = [end_effector,obj.end_effectors];
      input_frame = MultiCoordinateFrame({end_effector.frame,obj.getInputFrame.frame{:}});
      obj = obj.setNumInputs(obj.getNumInputs+end_effector.frame.dim);
      obj = obj.setInputFrame(input_frame);
    end
    
    function q_d0 = getInitialState(obj)
      q_d0 = obj.q_d0;
    end
        
    function q_dn = mimoUpdate(obj,t,q_d,varargin)
      
      num_ee = length(obj.end_effectors);
      for i=1:num_ee
        ee_goals{i} = varargin{i};
      end
      supports = varargin{num_ee+1};
      com_goal = varargin{num_ee+2};
      q_nom = varargin{num_ee+3};
      x = varargin{num_ee+4};

      nq = obj.manip.getNumDOF();
      q = x(1:nq);
      st = obj.getSampleTime();
      dt = st(1);
      
      % get support contact Jacobian
      kinsol = doKinematics(obj.manip,q,false);
      active_supports = find(supports~=0);
      if (isempty(active_supports))
        error('No supporting bodies --> not standing!');
      end
      count=0;
      Jc = zeros(length(obj.manip.getBodyContacts(active_supports))*3,obj.manip.getNumDOF());
      for i=1:length(active_supports)
        nC = size(obj.manip.getBodyContacts(active_supports(i)),2);
        if nC>0
          body_points = zeros(3,nC);
          for j=1:nC,
            body_points(:,i) = obj.manip.getBody(active_supports(i)).getContactShapes{j}.getPoints;
          end
          [~,Jc(3*count+(1:3*nC),:)] = obj.manip.forwardKin(kinsol,active_supports(i),body_points);
          count = count + nC;
        end
      end
 
      gc = obj.manip.contactPositions(q);
      [cm,Jcm] = obj.manip.getCOM(q);
      
      Jp = Jc(:,1:6);
      Ja = Jc(:,7:end);
      Pq_qa = [-pinv(Jp)*Ja;eye(nq-6)];

      % get support contact nullspace projection matrix
      Jc = Jc * Pq_qa;
      Nc = eye(nq-6) - pinv(Jc)*Jc;
      
      % COM projection matrix
      P = eye(3);% diag([1 1 0]);
      
      % compute COM error 
      err_com = P*(com_goal - cm);
      J_com = P*Jcm*Pq_qa;
      dq_com = obj.k_com * pinv(J_com) * err_com;
%       norm(dq_com)
%       if (norm(dq_com)>0.5)
%         dq_com = 0.5*dq_com/norm(dq_com);
%       end  
      % COM nullspace projection matrix
      Ncom = eye(nq-6) - pinv(J_com)*J_com;
 
      % debug
      colors = 'rgbyk';
      scope('Atlas','com_err',t,norm(err_com),struct('linespec',colors(1),'scope_id',1));

      % debugging by Russ
      scope('Atlas','com',cm(1),cm(2),struct('linespec','r','scope_id',2,'resetOnXval',false));
      scope('Atlas','com_goal',com_goal(1),com_goal(2),struct('linespec','go','scope_id',2,'resetOnXval',false,'num_points',1));
      for i=[1 2 4 3 1]
        scope('Atlas','leftfoot',gc(1,i),gc(2,i),struct('linespec','b*-','scope_id',2,'resetOnXval',false,'num_points',5));
      end
      for i=[5 6 8 7 5]
        scope('Atlas','rightfoot',gc(1,i),gc(2,i),struct('linespec','b*-','scope_id',2,'resetOnXval',false,'num_points',5));
      end
      % highlight points that are above/below the ground
      for i=1:8
        if gc(3,i) > 0.002
          scope('Atlas','raised_contacts',gc(1,i),gc(2,i),struct('linespec','g*','scope_id',2,'resetOnXval',false,'num_points',5));
        elseif gc(3,i) < -0.002
          scope('Atlas','sunken_contacts',gc(1,i),gc(2,i),struct('linespec','r*','scope_id',2,'resetOnXval',false,'num_points',5));
        end
      end
      
      % compute end effector deltas
      dq_ee = zeros(nq-6,1);
      N_ee{1} = eye(nq-6);
      for i=num_ee:-1:1
        active = (ee_goals{i}(1) ~= 0);
        if active
          ee_goal_i = ee_goals{i}(2:4);
          [ee,Jee] = obj.end_effectors(i).doKin(q);
          err_ee = ee_goal_i - ee;
          % debug
          scope('Atlas',strcat('ee_err ',int2str(i)),t,norm(err_ee),struct('linespec',colors(i+1),'scope_id',1));
          Pm = obj.end_effectors(i).P_mask;
          J1 = Jee*Pm*Pq_qa;
          dq_ee_i = N_ee{num_ee-i+1} * obj.end_effectors(i).gain * pinv(J1) * err_ee;
          if (norm(dq_ee_i)>obj.end_effectors(i).normbound)
            dq_ee_i = obj.end_effectors(i).normbound*dq_ee_i/norm(dq_ee_i);
          end  
          dq_ee = dq_ee + dq_ee_i;
          % assume end effectors are priortized in the order they 
          % are given for now
%           J2 = Jee*Pq_qa;
%           N_ee{num_ee-i+2} = N_ee{num_ee-i+1} * (eye(nq-6) - pinv(J2) * J2);
          N_ee{num_ee-i+2} = N_ee{num_ee-i+1} * (eye(nq-6) - pinv(J1) * J1);
        else
          N_ee{num_ee-i+2} = N_ee{num_ee-i+1};
        end
      end
      
      % cascaded end effector null space projection matrix
      Nee = N_ee{length(obj.end_effectors)+1};
      
      % compute nominal position error
      err_nom = q_nom - q(7:end);
      dq_nom = obj.k_nom * err_nom; 
      
      % debug
      %scope('Atlas',strcat('nom_err ',int2str(i)),t,norm(err_nom),struct('linespec','b--','scope_id',1));
          
      dq_des = Nc * (dq_com + Ncom*dq_ee + Ncom*Nee*dq_nom);
      
      % debug: dq_des should be in nullspace of ground contact jacobian
      %valuecheck(norm(Jc*Pq_qa*dq_des),0);
      
      % map to input frame
      B = obj.manip.getB();
      dq_des = B(7:end,:)' * dq_des;

      q_dn = q_d + dt*dq_des;
    end
    
    function y = mimoOutput(obj,t,q_d,varargin)
      y = q_d;
    end
    
    function obj = setInitialState(obj,q_d0)
      typecheck(q_d0,'double');
      sizecheck(q_d0,obj.getNumStates());
      obj.q_d0 = q_d0;
    end
    
  end
  
  properties
    manip
    k_nom = 0.15
    k_com = 0.5
    end_effectors = []
    q_d0 % initial state
    q_d_max
    q_d_min
  end
end