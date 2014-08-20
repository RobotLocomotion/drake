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
  command = [sprintf('export DYLD_FRAMEWORK_PATH=%s; ',getCMakeParam('DYLD_FRAMEWORK_PATH')), command];
  command = [sprintf('export DYLD_LIBRARY_PATH=%s; ',getCMakeParam('DYLD_LIBRARY_PATH')), command];
else
  command = [sprintf('export LD_LIBRARY_PATH=%s; ',getCMakeParam('LD_LIBRARY_PATH')), command];
end

[status,result] = system(command);
