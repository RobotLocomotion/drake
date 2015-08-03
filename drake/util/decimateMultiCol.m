function odata = decimateMultiCol(x,r,varargin)
num_cols = size(x,2);
odata = zeros(ceil(size(x,1)/r),num_cols);
for i = 1:num_cols
  odata(:,i) = decimate(x(:,i),r,varargin{:});
end
end