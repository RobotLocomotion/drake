function sizecheck(obj,sizemat)

% SIZECHECK
%   Usage:  sizecheck(obj,sizemat)

s = size(obj);
if (length(s)~=length(sizemat) || any(s~=sizemat))
  error(['Wrong size.  Expected [',num2str(sizemat),'] but got a [', num2str(s), '] instead.']);
end

