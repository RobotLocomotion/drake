function urdfs = allURDFs(rootdir)
% @retval urdfs a cell matrix with the names of all of the URDFs in found under the root directory
%
% This is useful, for instance, for unit tests which want to test some
% basic operation on every urdf (e.g. in drake)

if nargin < 1, rootdir = getDrakePath(); end

urdfs = {};

% Creates a command that finds all URDFs within rootdir. It uses the
% `find` program, which is assumed to be available.
command = ['find -L ', rootdir, ' -iname "*.urdf" '];

% Updates the above command to blacklist any URDF inside a dev/ subdirectory.
command = [command, '-not -path "', rootdir, '/*/dev/*" '];

% Updates the above command to blacklist any URDF containing the string
% "irb_140_convhull".
command = [command, '-not -path "', rootdir, '/*irb_140_convhull*" '];

% Updates the above command to blacklist any URDF whose full path contains the
% string "fixed_to_world".
% command = [command, '-not -path "', rootdir, '*fixed_to_world*" '];

% Updates the above command to blacklist any URDF whose full path contains the
% string "floating_in_world".
% command = [command, '-not -path "', rootdir, '*floating_in_world*" '];

% Finds URDF files using the previously defined command.
[info, p] = system(command);

if info == 0
  while ~isempty(p)
    [pt, p] = strtok(p);
%    pt=fileparts(pt);
    p = regexprep(p, [pt, '.*\n'], '', 'dotexceptnewline');
    urdfs = vertcat(urdfs, pt);
  end
else
  % If the `find` program fails for some reason (windows?), then print an error
  % message.
  error_msg = ['ERROR: drake/util/allURDFs.m: Failed to find any URDFs! '];
  error_msg = [error_msg, 'Ensure program `find` is installed and available.'];
  error('Drake:MissingDependency:find', error_msg);
end

