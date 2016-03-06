function baryTest

% note: also tests singleton dimensions
bins = { linspace(0,5,6), 1, linspace(0,10,11) };
pts = [5*rand(1,100); ones(1,100); 10*rand(1,100)];

[idx,coef,dcoef] = barycentricInterpolation(bins,pts);
d = size(coef,1);

X = cell(1,numel(bins));
[X{:}] = ndgrid(bins{:});
X = cellfun(@(a)reshape(a,1,[]),X,'UniformOutput',false);
gridpoints = vertcat(X{:});

for i=1:size(pts,2)
  pt = gridpoints(:,idx(:,i))*coef(:,i);
  valuecheck(pts(:,i),pt);

  dpt = gridpoints(:,idx(:,i))*dcoef(d*(i-1)+(1:d),:);
  valuecheck(dpt,diag([1 0 1])); % zero because of the singleton dimension
end
