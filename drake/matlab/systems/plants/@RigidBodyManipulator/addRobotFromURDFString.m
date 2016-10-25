function model=addRobotFromURDFString(model,urdf_string,varargin)
% Parses URDF from a string. Filename references must use the
% "package://" syntax, as there is no notion of a relative path here.
%
% @param urdf_string xml string containing the urdf 
%
% @options floating boolean where true means that a floating joint is
% automatically added to the root. @default false
% @options inertial boolean where true means parse dynamics parameters,
% false means skip them.  @default true
% @options visual boolean where true means parse graphics parameters, false
% means skip them.  @default true
% @options visual_geometry boolean where true means to extract the
% points from the visual geometries (might be very dense for meshes).
% Useful for extracting the 2D geometries later.  @default false
% @ingroup URDF Parsing

typecheck(urdf_string,'char');
urdf_filename = tempname;
file_id = fopen(urdf_filename,'w');
fprintf(file_id, '%s', urdf_string);
fclose(file_id);

model = addRobotFromURDF(model, urdf_filename, varargin{:});

delete(urdf_filename);
