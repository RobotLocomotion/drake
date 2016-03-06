function is_ht = isHT(T)
% check if T is a homogeneous transform matrix
try
  sizecheck(T,[4,4]);
  R = T(1:3,1:3);
  p = T(1:3,4);
  valuecheck(T(4,4),1);
  valuecheck(T(4,1:3),[0 0 0]);
  valuecheck(R*R',eye(3));
  if nargout == 1
    is_ht = true;
  end
catch ex
  if nargout == 1
    is_ht = false
  else
    rethrow(ex);
  end
end
end
