function makeROSCompatibleURDF(input_urdf_filename,output_urdf_filename)
% Drake supports urdf input beyond what ROS and other URDF readers
% support.  Additional drake xml tags will simply be ignored by other URDF
% parsers, but parameters and the ability to write matlab expressions in
% place of floats can cause problems.  This script will parse the
% parameters and write a new urdf which is compliant.
%
% @param input_urdf_filename string containing the filename
% @param output_urdf_filename optional string containing the output
% filename @default: input_urdf_filename with '_compatible' inserted in the name
% just before the urdf

if nargin<2,
  [~,~,ext] = fileparts(input_urdf_filename);
  output_urdf_filename = strrep(input_urdf_filename,ext,['_compatible',ext]);
end

param_db = struct();
urdf = xmlread(input_urdf_filename);

% parse parameters
parameters = urdf.getElementsByTagName('parameter');
for i=0:(parameters.getLength()-1)
  param_db = parseParameter(param_db,parameters.item(i));
end

% now iterate through ALL nodes and evaluate them based on their parameters
nodes = urdf.getElementsByTagName('*');
for i=0:(nodes.getLength()-1)
  attributes = nodes.item(i).getAttributes();
  for j=0:(attributes.getLength()-1)
    att = attributes.item(j);
    att.setValue(parseParamString(param_db,char(att.getValue())));
  end  
end

% delete the parameter nodes
while (parameters.getLength()>0)
  parent = parameters.item(0).getParentNode();
  parent.removeChild(parameters.item(0));
  parameters = urdf.getElementsByTagName('parameter');
end

xmlwrite(output_urdf_filename,urdf);
fprintf('Output written to %s\n',output_urdf_filename);

end


function param_db = parseParameter(param_db,node)
  name = char(node.getAttribute('name'));    % mandatory
  param_db.(name).value = str2num(char(node.getAttribute('value')));  % mandatory
  n = char(node.getAttribute('lb'));  % optional
  if isempty(n), param_db.(name).lb = -inf; else param_db.(name).lb = str2num(n); end
  n = char(node.getAttribute('ub'));  % optional
  if isempty(n), param_db.(name).ub = inf; else param_db.(name).ub = str2num(n); end
end

function default_value = parseParamString(param_db,str)
  pstr2 = regexprep(str,'\$(\w+)','param_db.(''$1'').value');
  try 
    default_value = eval(['[',pstr2,']']);
  catch
    default_value = pstr2;
    % intentionally fall through
  end
  if isnumeric(default_value)
    default_value = num2str(default_value,8);
  else
    default_value = pstr2;
  end
end
