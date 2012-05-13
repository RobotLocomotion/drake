classdef CoordinateFrame < handle
% Every input, state, and output in a RobotLibSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be 
% explicit about these coordinate systems when they make combinations of
% systems.
  
  properties (SetAccess=private,GetAccess=public)
    name;           % string name for this coordinate system
    dim;            % scalar dimension of this coordinate system
    transforms={};  % handles to CoordinateTransform objects

    prefix;         % string used as an automatic prefix for coordinates if they are not named manually
    coordinates={}; % list of coordinate names

    angle_flag=[];  % angle_flag(i)=true iff variable i wraps around 2pi
    poly;           % optional msspoly variables for this frame
  end
  
  methods
    function obj=CoordinateFrame(name,dim)
      typecheck(name,'char');
      obj.name = name;
      
      typecheck(dim,'double');
      sizecheck(dim,[1 1]);
      obj.dim = dim;
      
      ind = strfind(name,':');
      if isempty(ind)
        obj.prefix = name;
      else
        obj.prefix = name(ind(end)+1:end);
      end
      
      ind=1;
      function str=coordinateName(~);
        str=[obj.prefix,num2str(ind)];
        ind=ind+1;
      end
      obj.coordinates=cellfun(@coordinateName,cell(dim,1),'UniformOutput',false);
      
      if checkDependency('spot_enabled')
        if (obj.prefix(1)=='t') error('oops.  destined for a collision with msspoly representing time'); end
        obj.poly = msspoly(obj.prefix(1),dim);
      end
      
      obj.angle_flag = repmat(false,dim,1);
    end
    
    function obj=addTransform(obj,transform)
      typecheck(transform,'CoordinateTransform');
      if (getInputFrame(transform) ~= obj || getOutputFrame(transform) == obj)
        error('transform must be from this coordinate frame to another to be added');
      end
      if ~isempty(findTransform(obj,getOutputFrame(transform)))
        error('i already have a transform that gets me to that frame');
      end
      obj.transforms{end+1}=transform;
    end
    
    function tf=findTransform(obj,target)
      typecheck(target,'CoordinateFrame');
      ind=find(cellfun(@(a)getOutputFrame(a)==target,obj.transforms));
      if (isempty(ind))
        tf=[];
      else
        tf=obj.transforms{ind};
      end
    end
    
    function obj=setAngleFlags(obj,flags)
      typecheck(flags,'logical');
      sizecheck(flags,[obj.dim,1]);
      obj.angle_flag = flags;
    end
    
    function obj=setCoordinateNames(obj,cnames)
      if (iscell(cnames) && isvector(cnames) && length(cnames)==obj.dim && all(cellfun(@ischar,cnames)))
        obj.coordinates=cnames;
      else
        error('cnames must be a cell vector of length dim populated with strings'); 
      end
    end
  end
  
  methods (Static)
    function obj = getSingleton(name,dim)  
      % in some places, it may be enforce that only a single coordinate
      % frame exists with a given name. use this method to
      % create/access a singleton.  
      % 
      % @param name string name for the coordinate frame
      % @param dim dimension of the coordinate frame.  
      
      persistent objectStore;
      
      if (isempty(objectStore)) 
        objectStore={}; 
        ind=[];
      else
        ind=find(cellfun(@(frame)strcmp(frame.name,name),objectStore));
      end
      
      if isempty(ind)
        obj = CoordinateFrame(name,dim);
        objectStore{end+1}=obj;
      else
        if (length(ind)>1) error('found multiple matches in object store.  this shouldn''t happen'); end
        if (objectStore{ind}.dim ~= dim)
          error('found singleton with same name, but different dim.  this is not allowed'); 
        end
        obj = objectStore{ind};
      end
    end
  end
  % todo: consider putting LCM encode/decode in here
end
