function typcheck(obj,type)

% TYPECHECK
%   Usage:  typcheck(obj,type)
%   Checks if isa(obj,type), otherwise throws an error.  If type is a cell
%   array, then obj has to be type{1} OR type{2} OR ... 

  if (iscell(type))
    for i=1:length(type)
      t(i) = isa(obj,type{i});
    end
    if (~any(t))
      error(['Wrong type']);
    end
  else
    if (~isa(obj,type))
      error(['Wrong type.  Expected ',type,' but got a ', class(obj), ' instead.']);
    end
  end
  
end