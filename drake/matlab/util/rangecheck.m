function [tf,errstr]=rangecheck(obj,minval,maxval)

% checks obj element-wise to make sure that all values lie within the
% specified range.

tf = true;
errstr = '';

if any(obj(:)<minval)
  errstr = sprintf('Out of range.  Min val: %d but needed %d',min(obj(:)),minval);
  tf = false;
  if nargout > 0
    return;
  end
  error(errstr);
end

if any(obj(:)>maxval)
  errstr = sprintf('Out of range.  Max val: %d but needed %d',max(obj(:)),maxval);
  tf = false;
  if nargout > 0
    return;
  end
  error(errstr);
end

  