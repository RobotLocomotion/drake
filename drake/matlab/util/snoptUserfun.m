function [f,G] = snopt_userfun(x)
% I want to get away from using m files for the functions, so this file just evaluates a (global) function handle.  Ugh.

global SNOPT_USERFUN;
%try % snopt crashes if the userfun crashes
  [f,G] = SNOPT_USERFUN(x);
  G=full(G);  % it seems that snopt can't handle sparse matrices.
% catch
%   err=lasterror
%   for i=1:length(err.stack)
%     fprintf(1,'%s , line %d\n',err.stack(i).name,err.stack(i).line);
%   end
%   keyboard;  
% end
