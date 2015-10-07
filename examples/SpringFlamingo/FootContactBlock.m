classdef FootContactBlock < MIMODrakeSystem
	properties
    mex_ptr;
    controller_data;
    lfoot_idx;
    rfoot_idx;
    contact_threshold; % min height above terrain to be considered in contact
    use_contact_logic_OR;
  end
  
  methods
    function obj = FootContactBlock(r,controller_data,options)
      % @param r rigid body manipulator instance
      % @param controller_data FullStateQPControllerData object 
      % @param options struct
      % @option contact_threshold - minimum height above terrain for points to be in contact
      % @option use_contact_logic_OR - only applies for time-varying case
      %         if false: always do a logical AND with planned support 
      %           and sensed support
      %         if true: do logical OR with planned support and sensed 
      %           support except when breaking contact

      typecheck(controller_data,'FullStateQPControllerData');
       
      if nargin<3
        options = struct();
      end
      
      input_frame = getStateFrame(r);
      output_frame = FootContactState;
      
      obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
      obj = setInputFrame(obj,input_frame);
      obj = setOutputFrame(obj,output_frame);
			
      if isfield(options,'contact_threshold')
        % minimum height above terrain for points to be in contact
        typecheck(options.contact_threshold,'double');
        sizecheck(options.contact_threshold,[1 1]);
        obj.contact_threshold = options.contact_threshold;
      else
        obj.contact_threshold = 0.001;
      end
      
      if isfield(options,'use_contact_logic_OR')
        % only applies for time-varying case
        % false: always do a logical AND with planned support and sensed support
        % true: do logical OR with planned support and sensed support
        % except when breaking contact
        typecheck(options.use_contact_logic_OR,'logical');
        sizecheck(options.use_contact_logic_OR,[1 1]);
        obj.use_contact_logic_OR = options.use_contact_logic_OR;
      else
        obj.use_contact_logic_OR = false;
      end
      
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate

      obj.controller_data = controller_data;
           
      if exist('supportDetectmex','file')~=3
        error('can''t find supportDetectmex.  did you build it?');
      end      
      obj.mex_ptr = SharedDataHandle(supportDetectmex(0,r.getMexModelPtr.ptr,0));
   
      if isfield(options,'left_foot_name')
        obj.lfoot_idx = findLinkId(r,options.left_foot_name);
      else
        obj.lfoot_idx = findLinkId(r,'left_foot');
      end
      
      if isfield(options,'right_foot_name')
        obj.rfoot_idx = findLinkId(r,options.right_foot_name);
      else
        obj.rfoot_idx = findLinkId(r,'right_foot');
      end
    end
   
    function y=mimoOutput(obj,t,~,x)      
      ctrl_data = obj.controller_data;

      contact_logic_AND = true;
      if (ctrl_data.lqr_is_time_varying)
        % extract current desired supports
        supp_idx = find(ctrl_data.support_times<=t,1,'last');
        supp = ctrl_data.supports(supp_idx);      
        if obj.use_contact_logic_OR && supp_idx > 1
          supp_prev = ctrl_data.supports(supp_idx-1);
          breaking_contact = length(supp_prev.bodies)>length(supp.bodies);
          contact_logic_AND = breaking_contact;
        end  
      else      
        supp = ctrl_data.supports;
      end
      
      % contact_sensor = -1 (no info), 0 (info, no contact), 1 (info, yes contact)
      contact_sensor=-1+0*supp.bodies;  % initialize to -1 for all
      
      active_supports = supportDetectmex(obj.mex_ptr.data,x,supp,contact_sensor,obj.contact_threshold,0,contact_logic_AND);

      y = [1.0*any(active_supports==obj.lfoot_idx); 1.0*any(active_supports==obj.rfoot_idx)];
		end
  end
  
end
