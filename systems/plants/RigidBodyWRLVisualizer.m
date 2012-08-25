classdef RigidBodyWRLVisualizer < Visualizer 
  % Loads a URDF file and implements the draw function.  
  % You could think of this as a very limited version of ROS' RViz. 
  % 
  
  methods
    function obj = RigidBodyWRLVisualizer(frame,model)
      checkDependency('vrml_enabled');
      
      obj=obj@Visualizer(frame);
      
      if (nargin<1)
        [filename,pathname]=uigetfile('*.urdf');
        obj.model = RigidBodyModel(fullfile(pathname,filename));
      elseif ischar(model)
        obj.model = RigidBodyModel(model);
      elseif isa(model,'RigidBodyModel')  
        obj.model = model;
      else
        error('model must be a RigidBodyModel or the name of a urdf file'); 
      end
      
      wrlfile = fullfile(tempdir,[obj.model.name,'.wrl']);
      obj.model.writeWRL(wrlfile);
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

  properties
    model;
    wrl;
  end
end
