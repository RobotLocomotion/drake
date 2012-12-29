function writeWRL(model,wrlfile,options)
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
if ~isfield(options,'ground') options.ground = false; end

fp = fopen(wrlfile,'w');

% write header
fprintf(fp,'#VRML V2.0 utf8\n\n');
fprintf(fp,'## ------------------------------------------------- ##\n');
fprintf(fp,'## This file was automatically generated using Drake ##\n');
fprintf(fp,'##   EDITING THIS FILE BY HAND IS NOT RECOMMENDED    ##\n');
fprintf(fp,'## ------------------------------------------------- ##\n\n');

% write default background color  % todo: get this from urdf?
fprintf(fp,'Background {\n\tskyColor 1 1 1\n}\n\n');

if (options.ground)
  m=20; n=20;
  color1 = [204 102 0]/256;  % csail orange
  color1 = hex2dec({'ee','cb','ad'})/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/)
  color2 = hex2dec({'cd','af','95'})/256;
  fprintf(fp,'Transform {\n  translation %f %f 0\n  rotation 1 0 0 1.5708\n  children [\n',-m/2,n/2);
  fprintf(fp,'Shape { geometry ElevationGrid {\n');
  %        fprintf(fp,'  solid "false"\n');
  fprintf(fp,'  xDimension %d\n',m);
  fprintf(fp,'  zDimension %d\n',n);
  fprintf(fp,'  height [');
  fprintf(fp,' %d', zeros(m,n));
  fprintf(fp,' ]\n');
  fprintf(fp,'  colorPerVertex FALSE\n');
  fprintf(fp,'   color Color { color [');
  for i=1:(m-1)
    for j=1:(n-1)
      if rem(i+j,2)==0
        fprintf(fp,' %.1f %.1f %.1f,', color1);
      else
        fprintf(fp,' %.1f %.1f %.1f,', color2);
      end
    end
  end
  fprintf(fp,'] }\n');
  fprintf(fp,'}\n}\n]\n}\n\n');
end

% write default viewpoints
if (options.ground)
  fprintf(fp,'Viewpoint {\n\tdescription "right"\n\tposition 0 -4 2\n\torientation 1 0 0 1.3\n}\n\n');
  fprintf(fp,'Transform {\n\trotation 0 1 0 1.3\n\tchildren Viewpoint {\n\tdescription "front"\n\tposition -1 0 5\n\torientation 0 0 1 1.5708\n}\n}\n\n');
  fprintf(fp,'Viewpoint {\n\tdescription "top"\n\tposition 0 0 4\n}\n\n');
else
  fprintf(fp,'Viewpoint {\n\tdescription "right"\n\tposition 0 -4 0\n\torientation 1 0 0 1.5708\n}\n\n');
  fprintf(fp,'Transform {\n\trotation 0 1 0 1.5708\n\tchildren Viewpoint {\n\tdescription "front"\n\tposition 0 0 4\n\torientation 0 0 1 1.5708\n}\n}\n\n');
  fprintf(fp,'Viewpoint {\n\tdescription "top"\n\tposition 0 0 4\n}\n\n');
end
% loop through bodies
for i=1:length(model.body)
  if isempty(model.body(i).parent)
    writeWRLBodyAndChildren(model,model.body(i),fp);
  end
end
end

function writeWRLBodyAndChildren(model,body,fp,td)
if (nargin<4) td=0; end % tab depth
  function tabprintf(varargin), for i=1:td, fprintf(fp,'\t'); end, fprintf(fp,varargin{:}); end

if ~isempty(body.wrljoint)
  fprintf(fp,'Transform {\n%s\n\tchildren [\n',body.wrljoint);
end

% if there is a a joint between the parent and the body, add it here
if ~isempty(body.parent)
  writeWRLJoint(body,fp);
  tabprintf('children [\n'); td=td+1;
end
td = writeWRLBody(body,fp,td);
for i=1:length(model.body)
  if (model.body(i).parent == body)
    writeWRLBodyAndChildren(model,model.body(i),fp,td);
  end
end
if ~isempty(body.parent)
  td=td-1; tabprintf(']\n');
  td=td-1; tabprintf('}\n'); % end Transform {
end

if ~isempty(body.wrljoint)
  % close brackets that were added during removal of fixed joints
  brac=body.wrljoint(body.wrljoint=='{'|body.wrljoint=='[');
  brac=regexprep(brac(end:-1:1),'[',']');
  brac=regexprep(brac,'{','}');
  
  fprintf(fp,']\n%s\n}\n',brac); % end wrljoint transform
end
end
