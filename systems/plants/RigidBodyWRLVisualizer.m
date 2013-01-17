classdef RigidBodyWRLVisualizer < Visualizer 
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  
  methods
    function obj = RigidBodyWRLVisualizer(manip,options)
      % @option ground set options.ground = true to have ground visualized

      checkDependency('vrml_enabled');
      typecheck(manip,'RigidBodyManipulator');
      
      obj=obj@Visualizer(manip.getStateFrame());
      obj.model = manip;
      
      if nargin<2
        options = struct();
      end
      if ~isfield(options,'ground') options.ground = manip.num_contacts>0; end
      
      wrlfile = fullfile(tempdir,[obj.model.name,'.wrl']);
      obj.model.writeWRL(wrlfile,options);
      obj.wrl = vrworld(wrlfile);
      if ~strcmpi(get(obj.wrl,'Open'),'on')
        open(obj.wrl);
      end
      if get(obj.wrl,'Clients')<1
        view(obj.wrl);
      end
      obj.display_time=false;
    end
    
    function delete(obj)
      close(obj.wrl);
      delete(obj.wrl);
    end
    
    function draw(obj,t,x)
      for i=1:length(obj.model.body)
        b = obj.model.body(i);
        if ~isempty(b.parent)
          node=getfield(obj.wrl,b.jointname);
          if (b.pitch==0)
            node.rotation=[b.joint_axis' x(i-1)];
          elseif isinf(b.pitch)
            node.translation=x(i-1)*b.joint_axis';
          else
            error('helical joints not implemented yet (but would be simple)');
          end
        end
      end
      set(obj.wrl,'Time',t);
      vrdrawnow;
    end
    
    function playbackAVI(varargin)
      error('AVI playback not implemented yet for VRML visualizers.  But should be possible');
    end
    
    function playbackSWF(varargin)
      error('SWF playback not available for VRML visualizers.  The vector graphics equivalent is playbackVRML.');
    end
    
    function playbackVRML(varargin)
      error('not implemented yet, but should be possible to record the sequence to a VRML movie');
    end
  end

  properties (Access=protected)
    model;
    wrl;
  end
end
