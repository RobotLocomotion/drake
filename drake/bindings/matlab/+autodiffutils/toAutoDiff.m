function obj = toAutoDiff(value, derivatives)

if nargin < 2
  derivatives = eye(numel(value));
end

if size(value, 2) == 1
  if size(value, 1) <= 73
    obj = autodiffutils.AutoDiffVectorMax73(value, derivatives);
  else
    obj = autodiffutils.AutoDiffVectorDynamic(value, derivatives);
  end
else
  obj = autodiffutils.AutoDiffMatrixDynamic(value, derivatives);
end

end

