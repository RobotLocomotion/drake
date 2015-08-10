function fevalPackageSafe(f)
% Run a function or script, which may be inside a Matlab package. This function assumes
% that the current directory has been set to the folder containing the given function
% or script, as is the case in all of our unit testing tools. It will temporarily 
% modify the working directory and the functio name in order to correctly call Matlab
% package functions, which cannot normally be called from the folder that contains them
% (and must instead be called with the entire package name prefixed).

[pathstr, name] = fileparts(pwd());
if strcmp(name(1), '+')
  % we're in a package
  % add the package name
  f = strcat(name(2:end), '.', f);
  % clean up our mess once we're done
  cleanup_handle = onCleanup(@() cd(name));
  % move up a directory and try again
  cd(pathstr)
  fevalPackageSafe(f);
else
  % not in a package, just evaluate
  feval(f);
end
