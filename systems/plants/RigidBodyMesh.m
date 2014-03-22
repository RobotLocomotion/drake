classdef RigidBodyMesh < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyMesh(filename)
      typecheck(filename,'char');
      obj.filename = filename;
    end
    
    function pts = getPoints(obj)
      [path,name,ext] = fileparts(obj.filename);
      wrlfile=[];
      if strcmpi(ext,'.stl')
        wrlfile = fullfile(tempdir,[name,'.wrl']);
        stl2vrml(fullfile(path,[name,ext]),tempdir);
      elseif strcmpi(ext,'.wrl')
        wrlfile = obj.filename;
      end
      if ~isempty(wrlfile)
        txt=fileread(wrlfile);
        
        ind=regexp(txt,'coordIndex[\s\n]*\[([^\]]*)\]','tokens'); ind = ind{1}{1};
        ind=strread(ind,'%d','delimiter',' ,')+1;
        
        pts=regexp(txt,'point[\s\n]*\[([^\]]*)\]','tokens'); pts = pts{1}{1};
        pts=strread(pts,'%f','delimiter',' ,');
        pts=reshape(pts,3,[]);
        pts=obj.T(1:end-1,:)*[pts;ones(1,size(pts,2))];

        n=max(diff(find(ind==0)));
        if (min(diff(find(ind==0)))~=n), error('need to handle this case'); end
        ind = reshape(ind,n,[]); ind(end,:)=[];
        pts = pts(:,ind);
      end
    end
    
    function shape = serializeToLCM(obj)
      shape = drake.lcmt_viewer_geometry_data();
      shape.type = shape.MESH;
      shape.string_data = obj.filename;
      shape.num_float_data = 1;
      shape.float_data = 1.0;  % scale

      shape.position = obj.T(1:3,4);
      shape.quaternion = rotmat2quat(obj.T(1:3,1:3));
      shape.color = [obj.c(:);1.0];
    end
  end
  
  properties
    filename;
  end
  
end