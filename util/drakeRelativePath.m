function full_path = drakeRelativePath(rel_path)
  % full_path = drakeRelativePath(rel_path) returns the full path
  % corresponding to a path given relative to the drake root
  %
  % @param rel_path - String specifying a path relative to the drake
  %   root.
  % 
  % @retval full_path - String containing the full path corresponding to
  %   rel_path
  %
  full_path = fullfile(getDrakePath(),rel_path);
end
