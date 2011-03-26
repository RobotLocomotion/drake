function sizecheck(obj,sizemat)

% SIZECHECK
%   Usage:  sizecheck(obj,sizemat)

s = size(obj);
if (length(sizemat)==1) 
  if(sizemat==0)
    if (all(s==[0 0])) return; end
    error(['Wrong size.  Expected [0 0], but got [', num2str(s), '] instead.']);
  elseif (length(s)~=2 || ~(all(s==[sizemat 1]) || all(s==[1 sizemat])))
    error(['Wrong size.  Expected [',num2str(sizemat),' 1] or [1 ', num2str(sizemat),'], but got [',num2str(s),'] intead.']);
  end
else
  if (length(s)~=length(sizemat) || any(s~=sizemat))
    error(['Wrong size.  Expected [',num2str(sizemat),'] but got a [', num2str(s), '] instead.']);
  end
end

