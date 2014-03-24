classdef RigidBodyMesh < RigidBodyGeometry
  
  methods 
    function obj = RigidBodyMesh(filename)
      obj = obj@RigidBodyGeometry(4);
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
      else
        error('unknown mesh file extension');
      end
      
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
    
    function writeWRLShape(obj,fp,td)
      function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end
      tabprintf(fp,'Transform {\n'); td=td+1;
      tabprintf(fp,'translation %f %f %f\n',obj.T(1:3,4));
      tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(obj.T(1:3,1:3)));

      [path,name,ext] = fileparts(obj.filename);
      wrlfile=[];
      if strcmpi(ext,'.stl')
        wrlfile = fullfile(tempdir,[name,'.wrl']);
        stl2vrml(fullfile(path,[name,ext]),tempdir);

        txt=fileread(wrlfile);
        txt = regexprep(txt,'#.*\n','','dotexceptnewline');
      
        tabprintf(fp,'children Shape {\n'); td=td+1;
        tabprintf(fp,'geometry %s',txt);
        tabprintf(fp,'appearance Appearance { material Material { diffuseColor %f %f %f } }\n',obj.c(1),obj.c(2),obj.c(3));
        td=td-1; tabprintf('}\n');
      elseif strcmpi(ext,'.wrl')
        wrlfile = obj.filename;
        
        txt=fileread(wrlfile);
        txt = regexprep(txt,'#.*\n','','dotexceptnewline');
      
        tabprintf(fp,'children [\n'); 
        fprintf(fp,'%s',txt); 
        tabprintf(fp,']\n'); % end children
      else
        error('unknown mesh file extension');
      end
      
      td=td-1; tabprintf(fp,'}\n'); % end Transform {
    end
    
  end
  
  properties
    filename;
  end
  
end