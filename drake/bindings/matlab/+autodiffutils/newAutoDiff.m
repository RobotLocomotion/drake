function obj = newAutoDiff(value, derivatives)

if nargin < 2
  derivatives = eye(numel(value));
end

if size(value, 2) == 1
  obj = autodiffutils.AutoDiffVectorDynamic(value, derivatives);
else
  obj = autodiffutils.AutoDiffMatrixDynamic(value, derivatives);
end

end

