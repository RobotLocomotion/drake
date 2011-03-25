function sizecheck(obj,sizemat)

% SIZECHECK
%   Usage:  sizecheck(obj,sizemat)

s = size(obj);
if (length(sizemat)==1) 
  if(sizemat==0) sizemat=[0 0];
  else sizemat = [sizemat, 1]; end
end
if (length(s)~=length(sizemat) || any(s~=sizemat))
  error(['Wrong size.  Expected [',num2str(sizemat),'] but got a [', num2str(s), '] instead.']);
end

