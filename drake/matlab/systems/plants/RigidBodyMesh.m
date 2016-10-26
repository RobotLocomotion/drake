classdef RigidBodyMesh < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyMesh(filename,varargin)
      % obj = RigidBodyMesh(filename) constructs a RigidBodyMesh object with
      % the geometry-to-body transform set to identity.
      %
      % obj = RigidBodyMesh(filename,T) constructs a RigidBodyMesh object with
      % the geometry-to-body transform T.
      % 
      % obj = RigidBodyMesh(filename,xyz,rpy) constructs a RigidBodyMesh
      % object with the geometry-to-body transform specified by the
      % position, xyz, and Euler angles, rpy.
      %
      % @param filename - string containing the full path to a .stl or
      %   .wrl file describing the mesh
      % @param T - 4x4 homogenous transform from geometry-frame to
      %   body-frame
      % @param xyz - 3-element vector specifying the position of the
      %   geometry in the body-frame
      % @param rpy - 3-element vector of Euler angles specifying the
      %   orientation of the geometry in the body-frame
      obj = obj@RigidBodyGeometry(4,varargin{:}); 
                                                  
      typecheck(filename,'char');
      obj.filename = filename;
    end
    
    function [pts,ind,normals] = loadFile(obj)
      obj = convertToWRL(obj);
      txt=fileread(obj.filename);
        
      pts=regexp(txt,'point[\s\n]*\[([^\]]*)\]','tokens'); pts = pts{1}{1};
      pts=strread(pts,'%f','delimiter',' ,');
      pts=diag(obj.scale)*reshape(pts,3,[]);
      pts=obj.T(1:3,:)*[pts;ones(1,size(pts,2))];

      if (nargout>1)
        ind=regexp(txt,'coordIndex[\s\n]*\[([^\]]*)\]','tokens'); ind = ind{1}{1};
        ind=strread(ind,'%d','delimiter',' ,')+1;
      end      
      
      if (nargout>2)
        normals=regexp(txt,'vector[\s\n]*\[([^\]]*)\]','tokens'); normals = normals{1}{1};
        normals=strread(normals,'%f','delimiter',' ,');
        normals=obj.T(1:3,1:3)*reshape(normals,3,[]);
      end
      
    end

    function pts = getBoundingBoxPoints(obj)
      % Return axis-aligned bounding-box vertices
      vertices = getPoints(obj);
      max_vals = repmat(max(vertices,[],2),1,8);
      min_vals = repmat(min(vertices,[],2),1,8);
      min_idx = logical([ 0 1 1 0 0 1 1 0;
                          1 1 1 1 0 0 0 0;
                          1 1 0 0 0 0 1 1]);
      pts = zeros(3,8);
      pts(min_idx) = min_vals(min_idx);
      pts(~min_idx) = max_vals(~min_idx);
      pts = obj.T(1:end-1,:)*[pts;ones(1,8)];
    end
    
    function pts = getPoints(obj)
%      assert(all(obj.scale == 1)); % todo: handle this case
      pts = loadFile(obj);
    end

    function geom = convertToWRL(geom)
      [path,name,ext] = fileparts(geom.filename);
      if ~exist(fullfile(path,[name,'.wrl']),'file')
        if strcmpi(ext,'.stl') || exist(fullfile(path,[name,'.stl']),'file')
          stl2vrml(fullfile(path,[name,'.stl']),path);
        else
          error(['missing mesh file or unknown file extension ',geom.filename]);
        end
      end
      geom.filename = fullfile(path,[name,'.wrl']);
    end
    
    function geom = convertToOBJ(geom)
      [path,name,ext] = fileparts(geom.filename);
      if ~strcmpi(ext,'.obj') && ~exist(fullfile(path,[name,'.obj']),'file')
        exe = ''; if ispc, exe = '.exe'; end
        converter = fullfile(pods_get_bin_path,['convert_to_obj',exe]);
        if exist(converter)
          systemWCMakeEnv([converter,' ',geom.filename]);
        else
          error('can''t convert %s to OBJ format',geom.filename);
        end
      end
      geom.filename = fullfile(path,[name,'.obj']);
    end
    
    function geometry = serializeToLCM(obj)
      obj = convertToOBJ(obj);
      geometry = drake.lcmt_viewer_geometry_data();
      geometry.type = geometry.MESH;
      geometry.string_data = obj.filename;
      geometry.num_float_data = length(obj.scale);
      geometry.float_data = obj.scale;  % scale

      geometry.position = obj.T(1:3,4);
      geometry.quaternion = rotmat2quat(obj.T(1:3,1:3));
      geometry.color = [obj.c(:);1.0];
    end
    
    function writeWRLShape(obj,fp,td)
      if isscalar(obj.scale) obj.scale = repmat(obj.scale,1,3); end
      assert(numel(obj.scale)==3);
      
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end
      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'scale %f %f %f\n',obj.scale);
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));

      obj = obj.convertToWRL();
      
      txt=fileread(obj.filename);
      txt = regexprep(txt,'#.*\n','','dotexceptnewline');
      
      tabprintf(fp,'children [\n');
      fprintf(fp,'%s',txt);
      tabprintf(fp,']\n'); % end children

      % add appearance info back in manually
      appearanceString = sprintf('appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
      txt = regexprep(txt,'geometry',[appearanceString,'geometry']);
      
      tabprintf(fp,'children [\n');
      fprintf(fp,'%s',txt);
      tabprintf(fp,']\n'); % end children
      
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end
    
  end
  
  properties
    filename;
    scale=[1 1 1];
  end
  
end
