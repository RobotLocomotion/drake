function [f,G] = snopt_userfun(x)
% I want to get away from using m files for the functions, so this file just evaluates a (global) function handle.  Ugh.

global SNOPT_USERFUN;
try % snopt crashes if the userfun crashes
  [f,G] = SNOPT_USERFUN(x);
catch
  lasterror
  keyboard;  
end