function tf = typecheck(obj,type)

% TYPECHECK
%   Usage:  typecheck(obj,type)
%   Checks if isa(obj,type), otherwise throws an error.  If type is a cell
%   array, then obj has to be type{1} OR type{2} OR ... 

tf = true;

if (iscell(type))
  for i=1:length(type)
    t(i) = isa(obj,type{i});
  end
  if (~any(t))
    tf = false;
    if (nargout<1)
      error(['Wrong type']);
    end
  end
else
  if (~isa(obj,type))
    tf = false;
    if (nargout<1)
      error(['Wrong type.  Expected ',type,' but got a ', class(obj), ' instead.']);
    end
  end
end

