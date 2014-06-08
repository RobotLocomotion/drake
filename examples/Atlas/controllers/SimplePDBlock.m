classdef SimplePDBlock < MIMODrakeSystem
% NOTEST
  % outputs a desired q_ddot (including floating dofs)
  properties
    nq;
    Kp;
    Kd;
    dt;
    robot;
    ctrl_data;
  end
  
  methods
    function obj = SimplePDBlock(r,controller_data,options)
      typecheck(r,'Atlas');
      if nargin > 1
        typecheck(controller_data,'SharedDataHandle');
      end
      
      coords = AtlasCoordinates(r);
      input_frame = MultiCoordinateFrame({coords,r.getStateFrame});
      obj = obj@MIMODrakeSystem(0,0,input_frame,coords,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,coords);

      obj.nq = getNumDOF(r);

      if nargin<3
        options = struct();
      end
      
      if isfield(options,'Kp')
        typecheck(options.Kp,'double');
        sizecheck(options.Kp,[obj.nq 1]);
        obj.Kp = options.Kp;
      else
        obj.Kp = 170.0*ones(obj.nq,1);
      end        
%       obj.Kp([1,2,6]) = 0; % ignore x,y,yaw
        
      if isfield(options,'Kd')
        typecheck(options.Kd,'double');
        sizecheck(options.Kd,[obj.nq 1]);
        obj.Kd = options.Kd;
      else
        obj.Kd = 19.0*ones(obj.nq,1);
      end
%       obj.Kd([1,2,6]) = 0; % ignore x,y,yaw
            
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        obj.dt = options.dt;
      else
        obj.dt = 0.001;
      end
      obj = setSampleTime(obj,[obj.dt;0]); % sets controller update rate
      
      obj.robot = r;
      if nargin > 1
        obj.ctrl_data = controller_data;
      end
    end
   
    function y=mimoOutput(obj,t,~,varargin)
      q_des = varargin{1};
      x = varargin{2};
      q = x(1:obj.nq);
      qd = x(obj.nq+1:end);
      
			err_q = [q_des(1:3)-q(1:3);angleDiff(q(4:end),q_des(4:end))];
      y = max(-100*ones(obj.nq,1),min(100*ones(obj.nq,1),obj.Kp.*err_q - obj.Kd.*qd));
    end
  end
  
end
