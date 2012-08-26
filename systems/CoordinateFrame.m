classdef CoordinateFrame < handle
% Every input, state, and output in a DynamicalSystem has a coordinate frame
% attached to it.  Many bugs can be avoided by forcing developers to be 
% explicit about these coordinate systems when they make combinations of
% systems.
  
  properties (SetAccess=private,GetAccess=public)
    name;           % string name for this coordinate system
    dim;            % scalar dimension of this coordinate system
    transforms={};  % handles to CoordinateTransform objects

    coordinates={}; % list of coordinate names

    angle_flag=[];  % angle_flag(i)=true iff variable i wraps around 2pi
    poly=[];        % optional msspoly variables for this frame
    prefix;
  end
  
  methods
    function obj=CoordinateFrame(name,dim,prefix)
      typecheck(name,'char');
      obj.name = name;
      
      typecheck(dim,'double');
      sizecheck(dim,[1 1]);
      obj.dim = dim;
      
      if (nargin<3)
        ind = strfind(name,':');
        if isempty(ind)
          prefix = name(1);
        else
          prefix = name(ind(end)+[1:2]);
        end
      else
        typecheck(prefix,'char');
        sizecheck(prefix,[1 1]);
      end
      obj.prefix = prefix;
      
      ind=1;
      function str=coordinateName(~);
        str=[prefix,num2str(ind)];
        ind=ind+1;
      end
      obj.coordinates=cellfun(@coordinateName,cell(dim,1),'UniformOutput',false);
      
      if checkDependency('spot_enabled') && dim>0
        if (prefix=='t') error('oops.  destined for a collision with msspoly representing time'); end
        obj.poly = msspoly(prefix,dim);
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
    
    function obj=updateTransform(obj,newtransform)
      % find a current transform with the same target frame and replace it
      % with the new transform
      typecheck(newtransform,'CoordinateTransform');
      ind=find(cellfun(@(a)getOutputFrame(a)==newtransform.getOutputFrame,obj.transforms));
      if (isempty(ind))
        error('no transform to update');
      elseif length(ind)>1
        error('found multiple transforms');
      else
        obj.transforms{ind}=newtransform;
      end
    end
    
    function [tf,loc]=ismember(obj,cell_of_frames)
      typecheck(cell_of_frames,'cell');
      loc=find(cellfun(@(a) a==obj,cell_of_frames));
      tf = ~isempty(loc);
    end
    
    function drawFrameGraph(obj)
      % depends on having graphviz2mat installed (from matlabcentral)
      % todo: make that a dependency in configure?

      A = 0;
      fr = {obj};
      [A,fr]=recurseTFs(obj,A,fr);
      
      function [A,fr]=recurseTFs(obj,A,fr)
        [~,ind]=ismember(obj,fr);
        children = cellfun(@getOutputFrame,obj.transforms,'UniformOutput',false);
        for i=1:length(children)
          [b,cind]=ismember(children{i},fr);
          if b
            A(ind,cind)=1;
            continue;
          else
            fr=vertcat(fr,children(i));
            A(ind,end+1)=1;
            [A,fr]=recurseTFs(children{i},A,fr);
          end
        end
      end
      
      if (size(A,1)<length(A)) A(length(A),end)=0; end
      if (size(A,2)<length(A)) A(end,length(A))=0; end
      drawGraph(A,cellfun(@(a) a.name,fr,'UniformOutput',false));
    end

    
    function [tf,options]=findTransform(obj,target,options)
      if (nargin<3) options=struct(); end
      if ~isfield(options,'throw_error_if_fail') options.throw_error_if_fail = false; end
      
      typecheck(target,'CoordinateFrame');
      ind=find(cellfun(@(a)getOutputFrame(a)==target,obj.transforms));
      if isempty(ind)
        if ~isfield(options,'depth') options.depth=inf; end
        if ~isfield(options,'dirty_list') options.dirty_list=[]; end
        if ~isfield(options,'queue') options.queue=[]; end
        if ~isfield(options,'tf_from_parent') options.tf_from_parent=[]; end

        tf=[];
        if (options.depth>1)
          options.depth = options.depth-1;

          % add myself to the dirty list
          options.dirty_list = horzcat(options.dirty_list, struct('frame',obj,'tf_from_parent',options.tf_from_parent));

          % add my children to the queue 
          options.queue = horzcat(options.queue,struct('frame',cellfun(@getOutputFrame,obj.transforms,'UniformOutput',false),'tf_from_parent',obj.transforms));
          
          % now check the queue for a match
          while isempty(tf) && ~isempty(options.queue)
            a = options.queue(1);
            options.queue = options.queue(2:end);
            options.tf_from_parent = a.tf_from_parent;

            if ismember(a.frame,{options.dirty_list(:).frame})
              continue;
            end
            
            [tf,options] = findTransform(a.frame,target,options);
          end
          
          if ~isempty(tf)  % then reconstruct tf chain
            parent = options.tf_from_parent;
            while ~isempty(parent)
              tf = cascade(parent,tf);
              [~,loc]=ismember(tf.getInputFrame,{options.dirty_list(:).frame});
              parent = options.dirty_list(loc).tf_from_parent;
            end
            return;
          end
        end
        
        if (nargin>2 && options.throw_error_if_fail)
          error(['Could not find any transform between ',obj.name,' and ', target.name]);
        end
      elseif length(ind)>1
        error(['Found multiple transforms between ',obj.name,' and ', target.name]);
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
    
    function fr=subFrame(obj,dims)
      if ~isnumeric(dims) || ~isvector(dims) error('dims must be a numeric vector'); end
      if (any(dims>obj.dim | dims<1)) error(['dims must be between 1 and ',obj.dim]); end
      fr = CoordinateFrame([obj.name,mat2str(dims)], length(dims), obj.prefix);
      fr.coordinates = obj.coordinates(dims);
      fr.angle_flag = obj.angle_flag(dims);
      fr.poly = obj.poly(dims);
    end
    
    function generateLCMType(obj)
      % writes an lcm type specification to file from the coordinate frame
      % description. not to be used frequently.
      
      name = lower(obj.name);
      typename = ['lcmt_',name];
      fname = [typename,'.lcm'];
      if strcmpi(input(['About to write file ',fname,' .  Should I proceed (y/n)? '],'s'),'y')
        fptr=fopen(fname,'w');
        
        fprintf(fptr,'// Note: this file was automatically generated using the\n// CoordinateFrame.generateLCMType() method.\n\n');
        fprintf(fptr,'struct %s\n{\n  int64_t timestamp;\n\n',typename);
        for i=1:obj.dim
          fprintf(fptr,'  double %s;\n',stripSpecialChars(obj.coordinates{i}));
        end
        fprintf(fptr,'}\n\n');
        fclose(fptr);
        disp('file written successfully.  done.');
      end
    end
  end

  methods (Static=true,Hidden=true)
    function s=stripSpecialChars(s)
      s=regexprep(s,'\\','');
    end
  end
  
  % todo: consider putting LCM encode/decode in here
end
