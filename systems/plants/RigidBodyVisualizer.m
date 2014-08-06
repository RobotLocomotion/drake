classdef RigidBodyVisualizer < Visualizer
  properties (Access=protected)
    model;
  end
  methods
    function obj = RigidBodyVisualizer(manip)
      obj = obj@Visualizer(getStateFrame(manip));
      obj.model = manip;
    end
    function inspector(obj,x0, state_dims,minrange,maxrange)
      if (nargin<2), x0 = getInitialState(obj.model); end
      if (nargin<3), state_dims = 1:getNumDOF(obj.model); end
      if any(state_dims)>getNumDOF(obj.model)
        error('can''t draw velocities');
      end
      [jlmin,jlmax] = getJointLimits(obj.model);
      jlmin(isinf(jlmin))=-2*pi; jlmax(isinf(jlmax))=2*pi;
      if (nargin<4), minrange = jlmin(state_dims); end
      if (nargin<5), maxrange = jlmax(state_dims); end
      
      inspector@Visualizer(obj,x0,state_dims,minrange,maxrange,obj.model);
    end
    
    function kinematicInspector(obj,body_or_frame_id,pt,q0,minrange,maxrange)
      % kinematicInspector(model,body_or_frame_id,pt,q0)
      % brings up a simple slider gui (like the inspector() in Visualizer)
      % which calls inverse kinematics to drive the specified point on the
      % robot through a Cartesian endpoint (when possible).
      % 
      % @param body_or_frame_id e.g. from findFrameId,findJointInd, or
      % findLinkInd
      % @param pt a 3x1 point in Cartesian space
      % @param q0 (optional) initial pose for the robot @default uses
      % getInitialState()
      %
      
      nq = getNumPositions(obj.model);
      if nargin<4
        x0 = getInitialState(obj.model);
      else
        x0 = [q0;zeros(getNumVelocities(obj.model),1)];
      end
      
      x = resolveConstraints(obj.model,x0);
      q0 = x(1:nq);
      q = q0; 
      
      obj.drawWrapper(0,x);
      
      kinsol = doKinematics(obj.model,q);
      desired_pt = forwardKin(obj.model,kinsol,body_or_frame_id,pt);

      if (nargin<5), minrange = desired_pt-repmat(5,3,1); end
      if (nargin<6), maxrange = desired_pt+repmat(5,3,1); end

      varnames = {'x','y','z'};
      rows = length(varnames);
      f = sfigure(98); clf;
      set(f, 'Position', [560 400 280 20 + 30*rows]);
      
      y=30*rows-10;
      for i=1:rows
        label{i} = uicontrol('Style','text','String',varnames{i}, ...
          'Position',[10+280*(i>rows), y+30*rows*(i>rows), 90, 20],'BackgroundColor',[.8 .8 .8]);
        slider{i} = uicontrol('Style', 'slider', 'Min', minrange(i), 'Max', maxrange(i), ...
          'Value', desired_pt(i), 'Position', [100+280*(i>rows), y+30*rows*(i>rows), 170, 20],...
          'Callback',{@update_display});

        % use a little undocumented matlab to get continuous slider feedback:
        slider_listener{i} = handle.listener(slider{i},'ActionEvent',@update_display);
        y = y - 30;
      end
      

      function update_display(source, eventdata)
        x = x0;

        for i=1:rows
          desired_pt(i) = get(slider{i}, 'Value');
        end
        ik_constraint = WorldPositionConstraint(obj.model,body_or_frame_id,pt,desired_pt,desired_pt);
        [q,info] = inverseKin(obj.model,q,q,ik_constraint);
        if info~=1, info, end
        x(1:nq) = q;
        
        obj.drawWrapper(0,x);
        desired_pt'
        q'
      end      
    end    
  end
end

