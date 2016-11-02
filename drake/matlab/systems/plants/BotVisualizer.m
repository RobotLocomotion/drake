classdef BotVisualizer < RigidBodyVisualizer
  % Wraps the visualizer functionality around the drake libbot visualizer
  % (externally compiled program).  
  % Note: unlike other visualizers, only one BotVisualizer window can be
  % open at a time (multiple BotVisualizer class instances will share the
  % same viewer)

  methods
%    constructor (with optional urdf arg and call to addRobotFromURDF) 
%        opens a new viewer app with unique ID (like the figure num)
%        specified at the command line
%        viewer_id should be a shared data handle, which sends a shutdown
%        command on delete
%    draw
%    playbackMovie
%    lcmglwrappers
    function obj = BotVisualizer(manip,use_collision_geometry)
      if nargin < 2, use_collision_geometry = false; end
      
      global g_disable_visualizers;
      if g_disable_visualizers % evaluates to false if empty
        error('Drake:MissingDependency:BotVisualizerDisabled','botvis is disabled with g_disable_visualizers');
      end
      
      checkDependency('lcm');
      typecheck(manip,'RigidBodyManipulator');
      
%      if numel(manip.urdf)~=1
%        error('Drake:BotVisualizer:UnsupportedModel','I don''t actually support robots with multiple urdfs yet, but it will be easy enough');
%      end

      obj = obj@RigidBodyVisualizer(manip);
      obj.use_collision_geometry = use_collision_geometry;

      lc = lcm.lcm.LCM.getSingleton();
      obj.status_agg = lcm.lcm.MessageAggregator();
      lc.subscribe('DRAKE_VIEWER_STATUS',obj.status_agg);

      % check if there is a viewer already running
      [~,ck] = system('ps ax 2> /dev/null | grep -i "drake-visualizer" | grep -c -v grep');
      if (str2num(ck)<1) 
        % try launching director first
        if exist(fullfile(pods_get_bin_path,'drake-visualizer'))
          disp('attempting to launch the drake director')
          retval = systemWCMakeEnv([fullfile(pods_get_bin_path,'drake-visualizer'),' &> drake-visualizer.out &']);

          if isempty(obj.status_agg.getNextMessage(10000)) % wait for viewer to come up
            type drake-visualizer.out
            error('Drake:BotVisualizer:AutostartFailed','Failed to automatically start up a viewer (or to receive the ack, see https://github.com/RobotLocomotion/drake/issues/317)');
          end
        end
      end
            
      obj = updateManipulator(obj,manip);
      
      obj.draw_msg = drake.lcmt_viewer_draw();
      nb = getNumBodies(manip);
      obj.draw_msg.num_links = nb;
      obj.draw_msg.link_name = {manip.body.linkname};
      obj.draw_msg.robot_num = [manip.body.robotnum];
      obj.draw_msg.position = single(zeros(nb,3));
      obj.draw_msg.quaternion = single(zeros(nb,4));
      
      draw(obj,0,getZeroConfiguration(manip));
    end
    
    function obj = updateManipulator(obj,manip)
      obj = updateManipulator@RigidBodyVisualizer(obj,manip);
      
      lc = lcm.lcm.LCM.getSingleton();
      vr = drake.lcmt_viewer_load_robot();
      vr.num_links = getNumBodies(manip);
      vr.link = javaArray('drake.lcmt_viewer_link_data',vr.num_links);
      for i=1:vr.num_links
        b = getBody(manip,i);
        link = drake.lcmt_viewer_link_data();
        link.name = b.linkname;
        link.robot_num = b.robotnum;
        if obj.use_collision_geometry
          link.num_geom = length(b.collision_geometry);
        else
          link.num_geom = length(b.visual_geometry);
        end
        if (link.num_geom>0)
          link.geom = javaArray('drake.lcmt_viewer_geometry_data',link.num_geom);
        end
        for j=1:link.num_geom
          if obj.use_collision_geometry
            link.geom(j) = serializeToLCM(b.collision_geometry{j});
          else
            link.geom(j) = serializeToLCM(b.visual_geometry{j});
          end
        end
        vr.link(i) = link;
      end
      
      lc.publish('DRAKE_VIEWER_LOAD_ROBOT',vr);
      
      if (false) % the message aggregator is missing valid acks
        % listen for acknowledgement
        ack = obj.status_agg.getNextMessage(5000);
        obj.status_agg.numMessagesAvailable()
        if isempty(ack)
          error('Drake:BotVisualizer:LoadRobotFailed','Did not receive ack from viewer');
        else
          msg = drake.lcmt_viewer_command(ack.data);
          %        if ~strcmp(vr.command_data,msg.command_data)
          %          error('Drake:BotVisualizer:LoadURDFFailed','ack from viewer contained different data');
          %        end
        end
      end      
    end
    
    function drawWrapper(obj,t,y)
      draw(obj,t,y);
    end
    
    function draw(obj,t,q)
      obj.draw_msg.timestamp = int64(t*1000000);
      
      q = q(1:obj.model.num_positions);  % be robust to people passing in the full state
      
      kinsol = doKinematics(obj.model,q);
      for i=1:getNumBodies(obj.model)
        pt = forwardKin(obj.model,kinsol,i,zeros(3,1),2);
        obj.draw_msg.position(i,:) = pt(1:3);
        obj.draw_msg.quaternion(i,:) = pt(4:7);
      end
      
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_DRAW',obj.draw_msg);
      
      if ~isempty(obj.lcmgl_inertia_ellipsoids)
      % http://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_ellipsoid
        for i=1:getNumBodies(obj.model)
          b = obj.model.body(i);
          if b.mass>0
            pt = forwardKin(obj.model,kinsol,i,b.com,2);
            R = quat2rotmat(pt(4:end));
            obj.lcmgl_inertia_ellipsoids.glPushMatrix();
            obj.lcmgl_inertia_ellipsoids.glTranslated(pt(1),pt(2),pt(3));
            [V,I] = eig(b.inertia);
            if (det(V)<0) V(:,1)=-V(:,1); end
            axis_angle = rotmat2axis(R*V);
            obj.lcmgl_inertia_ellipsoids.glRotated(180*axis_angle(4)/pi,axis_angle(1),axis_angle(2),axis_angle(3));
%            abc=1./sqrt(diag(I));  % poinsot's ellipsoid

            % my formulation: find an ellipsoid of constant density, 
            % obj.inertial_ellipsoids_density, with the same inertia
            %  find a,b,c,m,  subject to
            %       m/5 (b^2+c^2) = Ia
            %       m/5 (a^2+c^2) = Ib
            %       m/5 (a^2+b^2) = Ic
            %       m = obj.inertia_ellipsoids_density * 4*pi/3*a*b*c
            % which can be solved (in closed-form) iff the inertial triangle inequalities hold
            %       Ia < Ib+Ic, Ib < Ia+Ic, Ic < Ia+Ib
            % by observing that there exists a density rhohat for which
            % mhat = 5/2 and the solution ahat,bhat,chat is 
            %    ahat=sqrt(Ib+Ic-Ia), bhat=sqrt(Ia+Ic-Ib), chat=sqrt(Ib+Ic-Ia)
            % and then we can scale this ellipse to the match the desired
            % density using: a=k*ahat, b=k*bhat, c=k*chat. Solving for k
            % (handling the possibility that one of the diagonal 
            % elements of I could be zero. yields
            I = diag(I);
            abchat_sq = [-1 1 1; 1 -1 1; 1 1 -1]*I;
            if any(abchat_sq<0)
              abc = zeros(3,1);
            else
              abchat = sqrt(abchat_sq);
              k = nthroot(sum(I)/(2*sum(abchat_sq)*obj.inertia_ellipsoids_density*4/15*pi*prod(abchat)),5);
              abc = k*abchat;
            end
            
%            alternative scaling (also some form of equivalent ellipsoid) from
%            http://www.mathworks.com/help/physmod/sm/mech/vis/about-body-color-and-geometry.html
%            abc=real(sqrt(2*b.mass./(5*(trace(I)-2*diag(I))))); 

            obj.lcmgl_inertia_ellipsoids.glPushMatrix();
            obj.lcmgl_inertia_ellipsoids.glColor3f(0,0,1);
            obj.lcmgl_inertia_ellipsoids.glScalef(abc(1),abc(2),1e-5);
            obj.lcmgl_inertia_ellipsoids.sphere(zeros(3,1),1,20,20);
            obj.lcmgl_inertia_ellipsoids.glPopMatrix();
            obj.lcmgl_inertia_ellipsoids.glPushMatrix();
            obj.lcmgl_inertia_ellipsoids.glColor3f(0,1,0);
            obj.lcmgl_inertia_ellipsoids.glScalef(abc(1),1e-5,abc(3));
            obj.lcmgl_inertia_ellipsoids.sphere(zeros(3,1),1,20,20);
            obj.lcmgl_inertia_ellipsoids.glPopMatrix();
            obj.lcmgl_inertia_ellipsoids.glPushMatrix();
            obj.lcmgl_inertia_ellipsoids.glColor3f(1,0,0);
            obj.lcmgl_inertia_ellipsoids.glScalef(1e-5,abc(2),abc(3));
            obj.lcmgl_inertia_ellipsoids.sphere(zeros(3,1),1,20,20);
            obj.lcmgl_inertia_ellipsoids.glPopMatrix();
            obj.lcmgl_inertia_ellipsoids.glPopMatrix();
          end
        end
        
        obj.lcmgl_inertia_ellipsoids.switchBuffers();
      end
    end
    
    function obj = enableLCMGLInertiaEllipsoids(obj,density)
      % @param density a positive scalar where the ellipsoid drawn is the
      % the equivalent ellipsoid with the specified density.  @default is
      % the density of wood (walnut).
      checkDependency('lcmgl');
      obj.lcmgl_inertia_ellipsoids = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'Inertia Ellipsoid');
      if nargin>1
        obj.inertia_ellipsoids_density=density;
      end
    end
    
    function obj = loadRenderer(obj,renderer_dynobj_path)
      % dynamically load a libbot renderer
      vc = drake.lcmt_viewer_command();
      vc.command_type = vc.LOAD_RENDERER;
      vc.command_data = renderer_dynobj_path;
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
    end
    
    function playbackMovie(obj,xtraj,filename,options)
      ffmpeg = getCMakeParam('FFMPEG_EXECUTABLE');
      if isempty(ffmpeg)
        error('need ffmpeg.  rerun make configure from the prompt to help find it');
      end
      
      if (nargin<2)
        [filename,pathname] = uiputfile('*','Save playback to movie');
        if isequal(filename,0) || isequal(pathname,0)
          return;
        end
        filename = fullfile(pathname,filename);
      end
      
      [path,name,ext] = fileparts(filename);
      if isempty(ext), ext = '.mp4'; end  % set a default
      
      vc = drake.lcmt_viewer_command();
      vc.command_type = vc.START_RECORDING;
      vc.command_data = '';
      lc = lcm.lcm.LCM.getSingleton();
      lc.publish('DRAKE_VIEWER_COMMAND',vc);

      playback(obj,xtraj,options);
      
      vc.command_type = vc.STOP_RECORDING;
      lc.publish('DRAKE_VIEWER_COMMAND',vc);
      
      ppmsgz_filename='';
      while isempty(ppmsgz_filename)
        msg = obj.status_agg.getNextMessage(5000);
        if isempty(msg) % wait for ack
          error('Drake:BotVisualizer:RecordingFailed','Never received recording OK ack from viewer');
        end
        ppmsgz_filename=char(drake.lcmt_viewer_command(msg.data).command_data);
        % todo: make this more robust (so I don't get a different status
        % message)
      end
      
      system(['gzip -d ',ppmsgz_filename]);
      [p,n,e] = fileparts(ppmsgz_filename);  % remove .gz from filename
      ppms_filename = fullfile(p,n);

      vcodec = '';
      switch(ext)
        case '.mp4'
          vcodec = '-vcodec mpeg4'
        otherwise
          warning('haven''t handled this file extension yet.  you might need to add a -vcodec arg here to help ffmpeg decide');
      end
      system([ffmpeg,' -r 30 -y -vcodec ppm -f image2pipe -i ', ppms_filename,' ',vcodec,' ', filename]);
      delete(ppms_filename);
    end
    
  end
  
  properties 
    viewer_id;
    draw_msg;
    status_agg;
    use_collision_geometry=false;
    lcmgl_inertia_ellipsoids;
    inertia_ellipsoids_density=600; % kg/m^3 density of wood (walnut) 
  end
end
