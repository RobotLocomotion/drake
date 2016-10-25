classdef OcTree < handle
  % Wrapper class for octomap library octree functionality

  % Note: it does not HAVE TO be a handle class, but it holds persistent
  % data so conceptually it must be treated as such.  I think it's safer 
  % to make that explicit.
  
  properties (SetAccess=private)
    mex_ptr
    lcmgl
  end
  
  methods
    function obj = OcTree(resolution_or_filename)
      if ~exist('octomapWrapper','file')
        dep = 'octomap';
        error(['Drake:MissingDependency:',dep],['Cannot find required dependency: ',dep]);
      end
      if (nargin<1)
        error('Drake:OcTree:InvalidArguments','Usage: OcTree(resolution_or_filename)');
      end
      
      typecheck(resolution_or_filename,{'double','char'});
      obj.mex_ptr = octomapWrapper(resolution_or_filename);
      
    end

    function enableLCMGL(obj,lcmgl_channel)
      if (checkDependency('lcmgl'))
        obj.lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), lcmgl_channel);
      end
    end
    
    function updateNode(tree,pts,occupied)
      % @param pts a 3xn list of pts
      % @param occupied a 1xn logical which is true if pts(:,i) was
      %        observed to be occupied or false if it was observed 
      %        to be free. if occupied is a scalar, then all pts use the
      %        same value;
      
      n = size(pts,2);
      if n>1 && numel(occupied)==1, occupied = repmat(occupied,1,n); end
      occupied = logical(occupied);

      octomapWrapper(tree.mex_ptr,11,pts,occupied);
    end
    
    function occupancy_probability = search(tree,pts)
      occupancy_probability = octomapWrapper(tree.mex_ptr,1,pts);
    end
    
    function [nodes,values] = getLeafNodes(tree)
      [nodes,values] = octomapWrapper(tree.mex_ptr,2);
    end
    
    function insertPointCloud(tree,pointCloud,origin)
      octomapWrapper(tree.mex_ptr,12,pointCloud,origin);
    end
    
    function publishLCMGL(obj,threshold)
      if nargin<2, threshold = .5; end
      if ~isempty(obj.lcmgl) % fail quietly if dependency is missing
        [points,values,sizeVar] = octomapWrapper(obj.mex_ptr,2);
        points = points(:,values>threshold);
        sizeVar = sizeVar(values>threshold);
        sizeVar=repmat(sizeVar,3,1);
        obj.lcmgl.glColor3f(.5, .5, 1);
        
        for i=1:size(sizeVar,2)
          obj.lcmgl.box(points(:,i),sizeVar(:,i));
        end
        obj.lcmgl.switchBuffers();
      end
    end
  end
  
end
