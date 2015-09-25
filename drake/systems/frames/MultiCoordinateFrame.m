classdef MultiCoordinateFrame < CoordinateFrame
  % Makes a mux'd coordinate frame out of a list of other frames.  This is
  % the means by which drake supports (the simplest form of) multi-input
  % and multi-output systems.
  %
  % Note that, although the coordinate names (and polynomial variables) are
  % initialized to match the name of the sub coordinate frames, there is 
  % no contract guaranteeing that they will continue to match after 
  % subsequent coordinate renamings for the multi-frame or the sub-frames.
  
  properties (SetAccess=public,GetAccess=public)
    frame={};     % a list of CoordinateFrame objects
    frame_id=[];  % for each coordinate, an integer index into the frame 
                  % (from the list above) associated with that coordinate
    coord_ids={}; % for each frame, a list of associate coordinates   
    
    % speed optimization
    cell_vals={};
  end

  methods
    function obj = MultiCoordinateFrame(coordinate_frames,frame_id)
      if (nargin<2) frame_id = []; end
      frame_id = reshape(frame_id, [], 1);
      
      typecheck(coordinate_frames,'cell');
      name=[];
      dim=0;
      prefix=[];
      coordinates={};
      
      if (length(coordinate_frames)==1)
        error('you''ve only passed in a single frame.  you should be calling MultiCoordinateFrame.constructFrame instead, to get the correct behavior');
      end
      
      if isempty(frame_id)
        for i=1:length(coordinate_frames)
          frame_id = vertcat(frame_id,repmat(i,coordinate_frames{i}.dim,1));
        end
      else
        rangecheck(frame_id,1,length(coordinate_frames));
      end
      
      for i=1:length(coordinate_frames)
        typecheck(coordinate_frames{i},'CoordinateFrame');
        name = [name,'+',coordinate_frames{i}.name];
        dim = dim+coordinate_frames{i}.dim;
        prefix = vertcat(prefix,coordinate_frames{i}.prefix);
        coordinates(frame_id==i) = reshape(coordinate_frames{i}.getCoordinateNames(),[],1);
        coord_ids{i} = reshape(find(frame_id==i),1,[]);
      end
      name=name(2:end); % remove extra '+'
      
      sizecheck(frame_id,[dim,nan]);
      obj = obj@CoordinateFrame(name,dim,prefix,coordinates);
      obj.frame = coordinate_frames;
      obj.frame_id = frame_id;
      obj.coord_ids = reshape(coord_ids,1,[]);
      
      % preallocate
      obj.cell_vals = cell(1,length(obj.frame));
      for i=1:length(obj.frame)
        obj.cell_vals{i} = zeros(numel(obj.coord_ids{i}),1);
      end
    end
    
    function disp(obj)
      fprintf(1,'Multi-Coordinate Frame: %s (%d elements)\n',obj.name,obj.dim);
      for i=1:length(obj.frame)
        disp(obj.frame{i});
      end
    end
    
    function display(obj)
      fprintf(1,'Multi-Coordinate Frame: %s (%d elements)\n',obj.name,obj.dim);
      for i=1:length(obj.frame)
        fprintf(1,'  %s (%d elements)\n',obj.frame{i}.name, obj.frame{i}.dim);
      end
    end
    
    function tf = isequal_modulo_transforms(a,b)
      tf = isequal_modulo_transforms@CoordinateFrame(a,b) && ...
        isequal(a.frame_id,b.frame_id) && ...
        isequal(a.coord_ids,b.coord_ids) && ...
        isequal(size(a.frame),size(b.frame));
      for i=1:length(a.frame)
        tf = tf && isequal_modulo_transforms(a.frame{i},b.frame{i});
      end
    end
    
    function [tf,options] = findTransform(obj,target,options)
      % There are three ways to get a transform from this multiframe to
      % another frame.  One is if a transform exists directly from the
      % multi-frame.  Another is if the required transforms exist for ALL
      % of the child frames.  The third is if a transform exists from ONE
      % of the child frames to the entire target frame (e.g. if the target
      % is actually one of the child frames).  See the mimoCascade and
      % mimoFeedback methods for more complex combinations.

      if (nargin<3) options=struct(); end
      if ~isfield(options,'throw_error_if_fail') options.throw_error_if_fail = false; end

      throw_error_if_fail = options.throw_error_if_fail;
      options.throw_error_if_fail=false;
      [tf,options] = findTransform@CoordinateFrame(obj,target,options);

      if isempty(tf) && isa(target,'MultiCoordinateFrame')
        % see if there are transforms from all of the children to all of
        % the target children

        % this could only happen if the target is also a multiframe
        
        % handle the simple case first, where the number of subframes is 
        % the same and obj frame i has a transform to target frame i.
        tf=[];
        if getNumFrames(obj)==getNumFrames(target)
          for i=1:length(obj.frame)
            tfi = findTransform(obj.frame{i},getFrameByNum(target,i),options);
            if isempty(tfi)
              tf=[];
              fr2=getFrameByNum(target,i);
%              warning(['Could not find any transform between ',obj.frame{i}.name,' and ', fr2.name]);
              break;
            end  
            
            if isempty(tf)
              tf = tfi;
            else
              tf = parallel(tf,tfi);
            end
          end
          
          if ~isempty(tf)
            % prepend remapping from coordinates of parent to children
            T = sparse(1:obj.dim,[obj.coord_ids{:}],1,obj.dim,obj.dim);
            tf = cascade(AffineTransform(obj,tf.getInputFrame,T,zeros(obj.dim,1)),tf);
            tf = setOutputFrame(tf,target);  % because the parallel combination loop above will results in an input frame with a ladder of subframes.
          end
        end
      end
      
      if isempty(tf)
        % see if there is a transform from any ONE of the children to
        % the entire target
        [child_tf,fid]=findChildTransform(obj,target,options);
        if fid>0
          tf = transformToChild(obj,fid);
          if ~isempty(child_tf)
            tf = cascade(tf,child_tf);
          end
        end
      end

      if isempty(tf) && throw_error_if_fail
        error(['Could not find any transform between ',obj.name,' and ', target.name]);
      end
      options.throw_error_if_fail = throw_error_if_fail;
    end
    
    function tf = transformToChild(obj,frame_id)
      d = obj.frame{frame_id}.dim;
      T = sparse(1:d,obj.coord_ids{frame_id},1,d,obj.dim);
      tf = AffineTransform(obj,obj.frame{frame_id},T,zeros(d,1));
    end
    
    function [tf,fid] = findChildTransform(obj,target,options)
      % note: if target is one of the children, then this will return an
      % empty tf but an fid>0
      if (nargin<3) options=struct(); end
      childoptions = options;
      childoptions.throw_error_if_fail = false;
      tf=[];
      fid=-1;
      for i=1:length(obj.frame)
        if (obj.frame{i}==target)
          if (fid>0)
            error('Found transforms to this target from multiple children.  That''s not allowed');
          end
          fid = i;
        else
          thistf = findTransform(obj.frame{i},target,childoptions);
          if ~isempty(thistf)
            if fid<0
              tf = thistf;
              fid = i;
            else
              error('Found transforms to this target from multiple children.  That''s not allowed');
            end
          end
        end
      end
      if (fid<1 && options.throw_error_if_fail)
        error('Couldn''t find a transform from any of my children to this target');
      end       
    end

    function [tf,fid] = findChildReverseTransform(obj,origin,options)
      % finds a transform from origin to one of my children
      % note: if origin is one of the children, then this will return an
      % empty tf but an fid>0

      if (nargin<3) options=struct(); end
      childoptions = options;
      childoptions.throw_error_if_fail = false;
      tf=[];
      fid=-1;
      for i=1:length(obj.frame)
        if (obj.frame{i}==origin)
          if (fid>0)
            error('Found transforms from this origin to multiple children.  That''s not allowed');
          end
          fid = i;
        else
          thistf = findTransform(origin,obj.frame{i},childoptions);
          if ~isempty(thistf)
            if fid<0
              tf = thistf;
              fid = i;
            else
              error('Found transforms from this origin to multiple children.  That''s not allowed');
            end
          end
        end
      end
      if (fid<1 && options.throw_error_if_fail)
        error('Couldn''t find a transform from this origin to any of my children');
      end       
    end
    
    function fr = subFrameByFrameID(obj,frame_ids)
      % comparable to subFrame, but at the frame ID 
      
      if ~isnumeric(frame_ids) || ~isvector(frame_ids) error('frame_ids must be a numeric vector'); end
      if (any(frame_ids>length(obj.frame) | frame_ids<1)) error(['frame_ids must be between 1 and ',length(obj.frame)]); end
      if (length(frame_ids)==1)
        fr = obj.frame{frame_ids};
      else
        fr = MultiCoordinateFrame({obj.frame{frame_ids}});
      end
    end
    
    function ids = findSubFrameEquivalentModuloTransforms(obj,fr)
      ids=[];
      for i=1:length(obj.frame)
        if isequal_modulo_transforms(obj.frame{i},fr)
          ids(end+1) = i;
        end
      end
    end

    function varargout = splitCoordinates(obj,vector_vals)
      % Extract values of individual CoordinateFrames from a vector in the
      % multi-frame.  
      % @param vector_vals a vector in the multi-frame
      %  
      % Usage:  cv = splitCoordinates(obj,vv);
      %   a = cv{1}; b=cv{2};
      % or 
      %   [a,b] = splitCoordinates(obj,vv)
      
      for i=1:length(obj.frame)
        obj.cell_vals{i} = vector_vals(obj.coord_ids{i});
      end
      if nargout==1
        varargout = {obj.cell_vals};
      else
        varargout = obj.cell_vals;
      end
    end      
    
    function vector_vals = mergeCoordinates(obj,cell_vals)
%      sizecheck(cell_vals,length(obj.frame));  % commented out for speed
      vector_vals = cell_vals{1}(1)*zeros(obj.dim,1);
      for i=1:length(obj.frame)
        vector_vals(obj.coord_ids{i}) = cell_vals{i};
      end
    end
    
    function n = getNumFrames(obj)
      n = length(obj.frame);
    end

    function fr = getFrameByNum(obj,n)
      rangecheck(n,1,length(obj.frame));
      fr = obj.frame{n};
    end

    function fr = getFrameByName(obj,name)
      id = find(cellfun(@(a)strcmp(a.name,name),obj.frame));
      if length(id)~=1
        error(['couldn''t find unique frame named ',name]);
      end
      fr = obj.frame{id};
    end
    
    function fr = getFrameByNameRecursive(obj,name)
      % Looks in all subframes that are multicoordinate frames for this
      % frame.
      to_search = obj.frame;
      i = 1;
      while (i <= length(to_search))
        if (isa(to_search{i}, 'MultiCoordinateFrame'))
          to_search = [to_search to_search{i}.frame];
        end
        i = i + 1;
      end
      id = find(cellfun(@(a)strcmp(a.name,name),to_search));
      
      if length(id)~=1
        error(['couldn''t find unique frame named ',name]);
      end
      fr = to_search{id};
    end
    
    function id = getFrameNumByName(obj,name)
      id = find(cellfun(@(a)strcmp(a.name,name),obj.frame));
    end
    
    function id = getFrameNumByNameRecursive(obj,name)
      % Returns vector of subframe ids to get to the supplied
      % frame from the root. The first elem in the vec is
      % the root level subframe -- each sequential item in
      % the vec is the index into the next subframe down.
      to_search = obj.frame;
      frame_locations = num2cell(1:length(to_search));
      i = 1;
      while (i <= length(to_search))
        if (isa(to_search{i}, 'MultiCoordinateFrame'))
          to_search = [to_search to_search{i}.frame];
          for j=1:length(to_search{i}.frame)
            frame_locations{end+1} = [frame_locations{i} j];
          end
        end
        i = i + 1;
      end
      id = find(cellfun(@(a)strcmp(a.name,name),to_search));
      if length(id)>1
        error(['couldn''t find unique frame named ',name]);
      elseif length(id) == 1
        id = frame_locations{id};
      end
    end
    
    function id = getFrameNum(obj,frame)
      id = find(cellfun(@(a)isequal(a,frame),obj.frame));
      if length(id)>1
        error('frame matched multiple children.  child frames must be unique, otherwise the behavior could get confusing fast');
      end
    end

    function obj = replaceFrameNum(obj,num,new_subframe)
      if new_subframe.dim ~= obj.frame{num}.dim
        error('new subframe does not match the dimensions of the frame you are trying to replace');
      end
      obj.frame{num} = new_subframe;
    end
    
    function obj = appendFrame(obj,new_subframe)
      old_me = obj;
      newframes = obj.frame; newframes{end+1} = new_subframe;
      obj = obj.constructFrame(newframes);
      obj.frame_id(1:end-new_subframe.dim) = old_me.frame_id;
      obj.coord_ids(1:end-1) = old_me.coord_ids;
      % Make sure the vectors are all facing the same way...
      % In the drakeAtlasSimul case this was actually a problem...
      % maybe the frame coord_ids as initialized within Atlas
      % somewhere were set up as 1xN? MultiCoordFrame seems to 
      % generate Nx1 natively. But only 1xN is working for
      % later frame generation... I need to figure this out :P
      for i=1:length(obj.coord_ids)
        vec = obj.coord_ids{i};
        obj.coord_ids{i} = vec(:).';
      end
    end
    
    function str = getCoordinateName(obj,i)
      ind = obj.frame_id(i);
      str = getCoordinateName(obj.frame{ind},find(obj.coord_ids{ind}==i));
    end
    
    function insys=setupMultiInput(obj,mdl,subsys)
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))  % assume that the simple ordering is ok
        insys = [subsys,'inselector'];
        tmp = [obj.coord_ids{:}]; [~,ind] = sort(tmp);
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',insys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str(ind));
        add_line(mdl,[insys,'/1'],[subsys,'/1']);
        subsys = insys;
      end
      insys = [subsys,'mux'];
      add_block('simulink3/Signals & Systems/Mux',[mdl,'/',insys],'Inputs',mat2str(cellfun(@(a) a.dim,obj.frame)));
      add_line(mdl,[insys,'/1'],[subsys,'/1']);
    end
    
    function outsys=setupMultiOutput(obj,mdl,subsys)
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))
        outsys = [subsys,'outselector'];
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',outsys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str([obj.coord_ids{:}]));
        add_line(mdl,[subsys,'/1'],[outsys,'/1']);
        subsys = outsys;
      end      
      outsys = [subsys,'demux'];
      add_block('simulink3/Signals & Systems/Demux',[mdl,'/',outsys],'Outputs',mat2str(cellfun(@(a) a.dim,obj.frame)));
      add_line(mdl,[subsys,'/1'],[outsys,'/1']);
    end
    
    function setupLCMInputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,{'char','SimulinkModelHandle'});
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      if ~valuecheck(obj.frame_id,sort(obj.frame_id))
        insys = [subsys,'inselector'];
        tmp = [obj.coord_ids{:}]; [~,ind] = sort(tmp);
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',insys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str(ind));
        add_line(mdl,[insys,'/1'],[subsys,'/',num2str(subsys_portnum)]);
        subsys = insys; subsys_portnum=1;
      end      
      add_block('simulink3/Signals & Systems/Mux',[mdl,'/mux',uid],'Inputs',num2str(length(obj.frame)));
      for i=1:length(obj.frame)
        setupLCMInputs(obj.frame{i},mdl,['mux',uid],i,options);
      end
      add_line(mdl,['mux',uid,'/1'],[subsys,'/',num2str(subsys_portnum)]);
    end
    
    function setupLCMOutputs(obj,mdl,subsys,subsys_portnum,options)
      typecheck(mdl,{'char','SimulinkModelHandle'});
      typecheck(subsys,'char');
      uid = datestr(now,'MMSSFFF');
      if (nargin<4) subsys_portnum=1; end
      typecheck(subsys_portnum,'double'); 
      if ~valuecheck(obj.frame_id,sort(obj.frame_id)) 
        outsys = [subsys,'outselector'];
        add_block('simulink3/Signals & Systems/Selector',[mdl,'/',outsys],'InputPortWidth',num2str(obj.dim),'Indices',mat2str([obj.coord_ids{:}]));
        add_line(mdl,[subsys,'/',subsys_portnum],[outsys,'/1']);
        subsys = outsys; subsys_portnum=1;
      end      
      add_block('simulink3/Signals & Systems/Demux',[mdl,'/demux',uid],'Outputs',mat2str([obj.frame{:}.dim]));
      for i=1:length(obj.frame)
        setupLCMOutputs(obj.frame{i},mdl,['demux',uid],i,options);
      end
      add_line(mdl,[subsys,'/',num2str(subsys_portnum)],['demux',uid,'/1']);
    end    
    
  end
  
  methods (Access=protected)
    function [A,fr] = extractFrameGraph(obj)
      [A,fr] = extractFrameGraph@CoordinateFrame(obj);
      for i=1:length(obj.frame)
        if ~ismember(obj.frame{i},fr)
          [B,frb] = extractFrameGraph(obj.frame{i});
          A = blkdiag(A,B);
          fr = vertcat(fr,frb);
        end
      end
    end
  end
  
  methods (Static=true)
    function obj = constructFrame(frames,frame_ids,zap_empty_frames)
      % if frames has only a single element, then return it, otherwise
      % construct the multi-frame
      typecheck(frames,'cell');
      
      if (nargin<2) frame_ids =[]; end
      if (nargin<3) zap_empty_frames = false; end
      
      if (nargin>1 && zap_empty_frames)
        i=1;
        while (i<=length(frames))  % zap empty frames
          if (frames{i}.dim<1)
            if any(frame_ids==i) error('bad frame_ids'); end
            frames=frames([1:i-1,i+1:end]);
            frame_ids(frame_ids>i)=frame_ids(frame_ids>i)-1;  % note: written to be ok if frame_ids is empty
          else
            i=i+1;
          end
        end
      end
      
      if length(frames)<1
        obj = CoordinateFrame('EmptyFrame',0);
      elseif (length(frames)==1)
        obj = frames{1};
      else
        obj = MultiCoordinateFrame(frames,frame_ids);
      end
    end
  end
end
