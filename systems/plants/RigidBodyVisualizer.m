classdef RigidBodyVisualizer < Visualizer
  properties (Access=protected)
    model;
  end
  properties
    gravity_visual_magnitude=.25;
    debug = false;  % if true, draws extras, like the coordinate frames and COMs for each link
    xlim=[-2,2];
    ylim=[-2,2];
    zlim=[-2,2];
  end
  methods
    function obj = RigidBodyVisualizer(manip)
      obj = obj@Visualizer(getPositionFrame(manip));
      obj.model = manip;
      obj.preserve_view = true;
    end
    
    function obj = updateManipulator(obj,manip)
      obj.model = manip;
    end
    
    function inspector(obj,x0,state_dims,minrange,maxrange,options)
      % brings up a simple slider gui that displays the robot
      % in the specified state when possible. It also shows resulting
      % forces and torques if some of the specified states are velocities.
      %
      % @param x0 the initial state to display the robot in
      % @param state_dims are the indices of the states to show on the
      % slider. Including velocity states will display the forces and torques.
      % @param minrange is the lower bound for the sliders
      % @param maxrange is the upper bound for the sliders
      % @option gravity_visual_magnitude specifies the visual length of
      % the vector representing the gravitational force. Other force
      % visualizations are scaled accordingly.

      if (nargin<2), x0 = getInitialState(obj.model); end
      if (nargin<3), state_dims = 1:getNumPositions(obj.model); end
      [jlmin,jlmax] = getJointLimits(obj.model);
      q0 = x0(1:getNumPositions(obj.model));
      jlmin(isinf(jlmin))=q0(isinf(jlmin))-2*pi; jlmax(isinf(jlmax))=q0(isinf(jlmax))+2*pi;
      xmin = [jlmin;-100*ones(getNumVelocities(obj.model),1)];
      xmax = [jlmax;100*ones(getNumVelocities(obj.model),1)];
      if (nargin<4), minrange = xmin(state_dims); end
      if (nargin<5), maxrange = xmax(state_dims); end
      if (nargin<6), options = struct(); end
      if isfield(options,'gravity_visual_magnitude')
          obj.gravity_visual_magnitude = options.gravity_visual_magnitude;
      end

      inspector@Visualizer(obj,x0,state_dims,minrange,maxrange,obj.model);
    end

    function draw(obj,t,x)
      nq = obj.model.num_positions;
      q = x(1:nq); %qd=x(nq+(1:nq));
      kinsol = obj.model.doKinematics(q);

      % for debugging:
      %co = get(gca,'ColorOrder');
      %h = [];
      % end debugging

      for i=1:numel(obj.model.body)
        for j=1:numel(obj.model.body(i).visual_geometry)
          draw(obj.model.body(i).visual_geometry{j},obj.model,kinsol,i);
        end
        if (obj.debug) % draw extra debugging info
          body = obj.model.body(i);
          origin = forwardKin(obj.model,kinsol,i,[0;0;0]);
          plot(origin(1),origin(2),'k+');
          if (body.mass~=0)
            com = forwardKin(obj.model,kinsol,i,body.com);
            plot3(com(1),com(2),com(3),'ro');
            line([origin(1),com(1)],[origin(2),com(2)],[origin(3),com(3)],'Color','r');
          end
          if body.parent>0
            parent_origin = forwardKin(obj.model,kinsol,body.parent,[0;0;0]);
            line([origin(1),parent_origin(1)],[origin(2),parent_origin(2)],[origin(3),parent_origin(3)],'Color','k');
          end
        end
      end

      axis equal;
      if ~isempty(obj.xlim)
        xlim(obj.xlim);
      end
      if ~isempty(obj.ylim)
        ylim(obj.ylim);
      end
      if ~isempty(obj.axis)
        axis(obj.axis);
      end

      xlabel('x');
      ylabel('y');
      zlabel('z');
      title(['t = ', num2str(t,'%.2f') ' sec']);
    end

    function kinematicInspector(obj,body_or_frame_id,pt,q0,minrange,maxrange)
      % kinematicInspector(model,body_or_frame_id,pt,q0)
      % brings up a simple slider gui (like the inspector() in Visualizer)
      % which calls inverse kinematics to drive the specified point on the
      % robot through a Cartesian endpoint (when possible).
      %
      % @param body_or_frame_id e.g. from findFrameId,findJointId, or
      % findLinkId
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
          'Position',[10+280*(i>rows), y+30*rows*(i>rows), 90, 20]);
        slider{i} = uicontrol('Style', 'slider', 'Min', minrange(i), 'Max', maxrange(i), ...
          'Value', desired_pt(i), 'Position', [100+280*(i>rows), y+30*rows*(i>rows), 170, 20],...
          'Callback',{@update_display});

        slider_listener{i} = addlistener(slider{i},'ContinuousValueChange',@update_display);
        y = y - 30;
      end


      function update_display(source, eventdata)
        if nargin>1 && isempty(eventdata), return; end  % was running twice for most events
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
