function [status,result] = systemWCmakeEnv(command)

% The input and output of this function are equivalent to 
% matlab's system function.  But since matlab messes with
% the shell environment (to the point where many apps fail 
% to run due to dynamic linking errors), I have the drake
% cmake script the following environment varibles:
%
%   LD_LIBRARY_PATH  (DYLD_LIBRARY_PATH and DYLD_FRAMEWORK_PATH on mac)
%
% to disk.  This function temporarily
% restores the values of those variables before calling 
% command, to help with execution.  

if (ismac)
  old_fw_path = getenv('DYLD_FRAMEWORK_PATH');
  old_lib_path = getenv('DYLD_LIBRARY_PATH');
  setenv('DYLD_FRAMEWORK_PATH',getCMakeParam('DYLD_FRAMEWORK_PATH'));
  setenv('DYLD_LIBRARY_PATH', getCMakeParam('DYLD_LIBRARY_PATH'));
  
  [status,result] = system(command);
  setenv('DYLD_FRAMEWORK_PATH', old_fw_path);
  setenv('DYLD_LIBRARY_PATH', old_lib_path);
else
  old_lib_path = getenv('LD_LIBRARY_PATH');
  setenv('LD_LIBRARY_PATH',getCMakeParam('LD_LIBRARY_PATH'));
  [status,result] = system(command);
  setenv('LD_LIBRARY_PATH', old_lib_path);
end
