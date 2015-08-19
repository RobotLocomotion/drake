function tf = sizecheck(obj,sizemat)

% SIZECHECK
%   Usage:  sizecheck(obj,sizemat)

s = size(obj);
tf = true;

if (ischar(sizemat))
  switch sizemat
    case 'colvec'
      if (length(s)~=2 || s(2)~=1)
        tf = false;
        if (nargout<1)
          error(['Wrong size.  Expected a column vector, but got [', num2str(s), '] instead.']);
        end
      end
    case 'rowvec'
      if (length(s)~=2 || s(1)~=1)
        tf = false;
        if (nargout<1)
          error(['Wrong size.  Expected a row vector, but got [', num2str(s), '] instead.']);
        end
      end
    otherwise
      error('Unsupported size %s',sizemat);
  end
elseif (length(sizemat)==1) 
  if(sizemat==0)
    if (all(s==[0 0])) return; end
    tf = false;
    if (nargout<1)
      error(['Wrong size.  Expected [0 0], but got [', num2str(s), '] instead.']);
    end
  elseif (length(s)~=2 || ~(all(s==[sizemat 1]) || all(s==[1 sizemat])))
    tf = false;
    if (nargout<1)
      error(['Wrong size.  Expected [',num2str(sizemat),' 1] or [1 ', num2str(sizemat),'], but got [',num2str(s),'] instead.']);
    end
  end
else
  docare = ~isnan(sizemat);
  if (length(sizemat)>length(s))  % pad with ones because matlab evaluates size(ones(3,2,1,1)) to [3,2]
    s = [s,ones(1,length(sizemat)-length(s))];
  end
  if (prod(sizemat(docare))>0 && (length(s)~=length(sizemat) || any(s(docare)~=sizemat(docare))))
    tf = false;
    if (nargout<1)
      error(['Wrong size.  Expected [',num2str(sizemat),'] but got a [', num2str(s), '] instead.']);
    end
  end
end

