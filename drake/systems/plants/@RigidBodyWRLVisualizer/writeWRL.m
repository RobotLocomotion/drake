function writeWRL(obj,wrlfile,options)

model = obj.model;

if nargin<2 || isempty(wrlfile)
  [filename,pathname]=uiputfile('*.wrl','Save WRL File',[model.name,'.wrl']);
  wrlfile=[pathname,filename];
else
  typecheck(wrlfile,'char');
  [path,name,ext]=fileparts(wrlfile);
  if isempty(ext)
    ext='.wrl';
  elseif ~strcmpi(ext,'.wrl');
    error('second argument should point to a wrl file (with extension .wrl)');
  end
end

if (nargin<3) options=struct(); end

fp = fopen(wrlfile,'w');

% write header
fprintf(fp,'#VRML V2.0 utf8\n\n');
fprintf(fp,'## ------------------------------------------------- ##\n');
fprintf(fp,'## This file was automatically generated using Drake ##\n');
fprintf(fp,'##   EDITING THIS FILE BY HAND IS NOT RECOMMENDED    ##\n');
fprintf(fp,'## ------------------------------------------------- ##\n\n');


fprintf(fp,'PointLight {\n');
fprintf(fp,'  on TRUE\n');
fprintf(fp,'  intensity 1\n');
fprintf(fp,'  location 0 0 20\n');
fprintf(fp,'  ambientIntensity 1\n');
fprintf(fp,'  color 1 1 1\n');
fprintf(fp,'}\n\n');

if isempty(model.terrain)
  fprintf(fp,'Background {\n\tskyColor 1 1 1\n}\n\n');
  % write default viewpoints
  fprintf(fp,'Viewpoint {\n\tdescription "right"\n\tposition 0 -4 0\n\torientation 1 0 0 1.5708\n}\n\n');
  fprintf(fp,'Transform {\n\trotation 0 1 0 1.5708\n\tchildren Viewpoint {\n\tdescription "front"\n\tposition 0 0 4\n\torientation 0 0 1 1.5708\n}\n}\n\n');
  fprintf(fp,'Viewpoint {\n\tdescription "top"\n\tposition 0 0 4\n}\n\n');
else
  %  model.terrain.writeWRL(fp);  % now gets written as a geometry
  fprintf(fp,'Background {\n\tskyColor 0.8818    0.9512    0.9941\n}\n\n');

  % write default viewpoints
  fprintf(fp,'Viewpoint {\n\tdescription "right"\n\tposition 0 -4 2\n\torientation 1 0 0 1.3\n}\n\n');
  fprintf(fp,'Transform {\n\trotation 0 1 0 1.3\n\tchildren Viewpoint {\n\tdescription "front"\n\tposition -1 0 5\n\torientation 0 0 1 1.5708\n}\n}\n\n');
  fprintf(fp,'Viewpoint {\n\tdescription "top"\n\tposition 0 0 4\n}\n\n');
end

% loop through bodies
for i=1:length(model.body)
  if model.body(i).parent<1
    writeWRLBodyAndChildren(model,i,fp);
  end
end
end

function writeWRLBodyAndChildren(model,body_ind,fp,td)
  if (nargin<4) td=0; end % tab depth
  function tabprintf(fp,varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

  body = model.body(body_ind);

  tabprintf(fp,'Transform {\n'); td=td+1;
  tabprintf(fp,'translation %f %f %f\n',body.Ttree(1:3,4));
  tabprintf(fp,'rotation %f %f %f %f\n',rotmat2axis(body.Ttree(1:3,1:3)));
  tabprintf(fp,'children [\n'); td=td+1;

  % if there is a a joint between the parent and the body, add it here
  if body.parent>0
    if isempty(body.jointname)
      tabprintf(fp,'Transform {\n'); td=td+1;
    else
      tabprintf(fp,'DEF %s Transform {\n',body.jointname); td=td+1;
    end
    if (body.floating)
      tabprintf(fp,'translation 0 0 0\n');
      tabprintf(fp,'rotation 0 1 0 0\n');
    elseif (body.pitch==0) % then it's a pin joint
      tabprintf(fp,'rotation 0 1 0 0\n');
    elseif isinf(body.pitch) % then it's a slider
      tabprintf(fp,'translation 0 0 0\n');
    end

    tabprintf(fp,'children [\n'); td=td+1;
  end

  for i=1:length(body.visual_geometry)
    writeWRLShape(body.visual_geometry{i},fp,td);
  end

  for i=1:length(model.body)
    if (model.body(i).parent == body_ind)
      writeWRLBodyAndChildren(model,i,fp,td);
    end
  end

  if body.parent>0
    td=td-1; tabprintf(fp,']\n'); % end children [
    td=td-1; tabprintf(fp,'}\n'); % end Transform {
  end

  td=td-1; tabprintf(fp,']\n'); % end children [
  td=td-1; tabprintf(fp,'}\n'); % end Transform {
end

    function wrlstr = parseWRLGeometry(node,wrl_appearance_str,model,robotnum,options)
      % param node DOM node for the geometry block
      % param X coordinate transform for the current body
      if (nargin<3) options=struct(); end
      if ~isfield(options,'urdfpath') options.urdfpath=[]; end

      wrlstr='';
      childNodes = node.getChildNodes();
      for i=1:childNodes.getLength()
        thisNode = childNodes.item(i-1);
        switch (lower(char(thisNode.getNodeName())))
          case 'box'
            s = parseParamString(model,robotnum,char(thisNode.getAttribute('size')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Box { size %f %f %f }\n\t%s}\n',s(1),s(2),s(3),wrl_appearance_str)];

          case 'cylinder'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            l = parseParamString(model,robotnum,char(thisNode.getAttribute('length')));

            % default axis for cylinder in urdf is the z-axis, but
            % the default in vrml is the y-axis.
            wrlstr=[wrlstr,sprintf('Transform {\n\trotation 1 0 0 1.5708\n\tchildren Shape {\n\t\tgeometry Cylinder {\n\t\t\theight %f\n\t\t\tradius %f\n\t\t}\n\t\t%s\n\t}\n}\n',l,r,wrl_appearance_str)];

          case 'sphere'
            r = parseParamString(model,robotnum,char(thisNode.getAttribute('radius')));
            wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry Sphere { radius %f }\n\t%s}\n',r,wrl_appearance_str)];

          case 'mesh'
            filename=char(thisNode.getAttribute('filename'));
            filename=RigidBody.parseMeshFilename(filename,options);
            [path,name,ext] = fileparts(filename);
            if strcmpi(ext,'.stl')
              wrlfile = fullfile(tempdir,[name,'.wrl']);
              stl2vrml(fullfile(path,[name,ext]),tempdir);
              txt=fileread(wrlfile);
              txt = regexprep(txt,'#.*\n','','dotexceptnewline');
              wrlstr=[wrlstr,sprintf('Shape {\n\tgeometry %s\n\t%s}\n',txt,wrl_appearance_str)];
%              wrlstr=[wrlstr,sprintf('Inline { url "%s" }\n',wrlfile)];
            elseif strcmpi(ext,'.wrl')
              txt = fileread(filename);
              txt = regexprep(txt,'#.*\n','','dotexceptnewline');
              wrlstr=[wrlstr,txt];
%              wrlstr=[wrlstr,sprintf('Inline { url "%s" }\n',filename)];
            end

          case {'#text','#comment'}
            % intentionally blank
          otherwise
            warning([char(thisNode.getNodeName()),' is not a supported element of robot/link/visual/material.']);
        end
      end

    end
