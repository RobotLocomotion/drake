function h = addpathTemporary(path_string_or_cell)
% addpathTemporary: Add a string or cell array of strings to the MATLAB path
% and automatically clean up afterward. 
% USAGE: 
% To use this function, you *must* assign its output to a local variable. The
% modifications to the path will persist only until that variable is 
% destroyed, typically when the function that contains it returns. If you do
% not assign the output to a variable, then the path changes will be 
% immediately undone when the next line of code executes. 
% 
% Typical usage might look like this:
%     path_handle = addpathTemporary(fullfile(getDrakePath, 'examples', 'ZMP'));
%
% @param path_string_or_cell a single path string or a cell array of strings
% @retval h the handle object which restores the path when it is destroyed

if ischar(path_string_or_cell)
  path_string_or_cell = {path_string_or_cell};
end

added_path = false(size(path_string_or_cell));
path_cell = regexp(path(), pathsep(), 'split');

for j = 1:length(path_string_or_cell)
  s = path_string_or_cell{j};
  if ~ismember(s, path_cell)
    addpath(path_string_or_cell{j});
    added_path(j) = true;
  end
end

h = onCleanup(@() cleanupPaths(path_string_or_cell(added_path)));
end

function cleanupPaths(paths_to_clear)
  for j = 1:length(paths_to_clear)
    rmpath(paths_to_clear{j})
  end
end

