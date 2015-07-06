classdef IKPDBlock < MIMODrakeSystem
  % outputs a desired q_ddot (including floating dofs)
  properties
    nq;
    Kp;
    Kd;
    controller_data; % pointer to shared data handle com/foot trajectories
    ikoptions;
    robot;
    max_nrm_err;
    input_foot_contacts;
    use_ik; % if false, just does PD on q_nom input
  end
  
  methods
    function obj = IKPDBlock(r,controller_data,options)
      typecheck(r,'Biped');
      if nargin > 1
        typecheck(controller_data,'QPControllerData');
      else
        controller_data = [];
      end
      
      if nargin<3
        options = struct();
      end

      if ~isfield(options,'input_foot_contacts')
        options.input_foot_contacts = false;
      else
        typecheck(options.input_foot_contacts,'logical');
      end
      
      coords = atlasFrames.AtlasCoordinates(r);
      if options.input_foot_contacts
        input_frame = MultiCoordinateFrame({coords,r.getStateFrame,atlasFrames.FootContactState});
      else
        input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      end
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.controller_data = controller_data;
      obj.nq = getNumPositions(r);
      obj.input_foot_contacts = options.input_foot_contacts;
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[obj.nq 1]);
        obj.Kp = options.Kp;
      else
        obj.Kp = 160.0*ones(obj.nq,1);
      end        
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[obj.nq 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = 19.0*ones(obj.nq,1);
      end

      if isfield(options,'use_ik')
        typecheck(options.use_ik,'logical');
        sizecheck(options.use_ik,1);
        obj.use_ik = options.use_ik;
      else
        obj.use_ik = true;
      end
 
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
            
      % setup IK parameters
      cost = Point(r.getStateFrame,1);
      cost.base_x = 0;
      cost.base_y = 0;
      cost.base_z = 0;
      cost.base_roll = 1000;
      cost.base_pitch = 1000;
      cost.base_yaw = 0;
      cost.back_bkz = 10;
      cost.back_bky = 10;
      cost.back_bkx = 10;
      cost.l_leg_hpz = 10;
      cost.r_leg_hpz = 10;
      cost.l_leg_kny = 5;
      cost.r_leg_kny = 5;
      cost = double(cost);
      
      obj.ikoptions = struct();
      obj.ikoptions.Q = diag(cost(1:obj.nq));

      % dofs to constrain to q_nom 
      if isfield(options,'fixed_dofs')
        typecheck(options.fixed_dofs,'double');
        obj.ikoptions.fixed_dofs = options.fixed_dofs;
      else
        obj.ikoptions.fixed_dofs = [];
      end
      
      obj.robot = r;
      obj.max_nrm_err = 1.5;      
    end
   
    function y=mimoOutput(obj,t,~,varargin)      
     
      q_nom = varargin{1};
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);      

      if ~obj.use_ik
        err_q = [q_nom(1:3)-q(1:3);angleDiff(q(4:end),q_nom(4:end))];
      else
        obj.ikoptions.q_nom = q_nom;        
        cdata = obj.controller_data;
        approx_args = {};
        for j = 1:length(cdata.link_constraints)
          if cdata.lqr_is_time_varying && ~isempty(cdata.link_constraints(j).coefs)
            pos = evaluateSplineInLinkConstraints(t,cdata.link_constraints, j);
            pos(1:3) = pos(1:3) - cdata.plan_shift;
            approx_args(end+1:end+3) = {cdata.link_constraints(j).link_ndx, cdata.link_constraints(j).pt, pos};
          elseif ~isempty(cdata.link_constraints(j).pos)
            pos = cdata.link_constraints(j).pos;
            approx_args(end+1:end+3) = {cdata.link_constraints(j).link_ndx, cdata.link_constraints(j).pt, pos};
          end
        end
        
        % note: we should really only try to control COM position when in
        % contact with the environment
        if cdata.lqr_is_time_varying
          com = fasteval(cdata.comtraj,t);
        else
          com = cdata.comtraj;
        end
        compos = [com(1:2) - cdata.plan_shift(1:2);nan];

        q_des = linearIK(obj.robot,q,0,compos,approx_args{:},obj.ikoptions);
        
        err_q = q_des - q;
        nrmerr = norm(err_q,1);
        if nrmerr > obj.max_nrm_err
          err_q = obj.max_nrm_err * err_q / nrmerr;
        end
      end

      y = max(-100*ones(obj.nq,1),min(100*ones(obj.nq,1),obj.Kp.*err_q - obj.Kd.*qd));
    end
  end
  
end
