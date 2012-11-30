function rangecheck(obj,minval,maxval)

% checks obj element-wise to make sure that all values lie within the
% specified range.

if any(obj(:)<minval)
  error('Out of range.  Min val: %d but needed %d',min(obj(:)),minval);
end

if any(obj(:)>maxval)
  error('Out of range.  Max val: %d but needed %d',max(obj(:)),maxval);
end

  