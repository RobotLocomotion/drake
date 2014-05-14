function full_path = drakeExamplePath(rel_path)
  % full_path = drakeRelativePath(rel_path) returns the full path
  % corresponding to a path given relative to the drake examples
  % directory 
  %
  % @param rel_path - String specifying a path relative to the drake
  %   examples directory
  % 
  % @retval full_path - String containing the full path corresponding to
  %   rel_path
  %
  full_path = drakeRelativePath(fullfile('examples',rel_path));
end
