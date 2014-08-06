function [c,dc] = crossSum(r1,r2)
% for 3 x N matrix r1 and 3 x N matrix r2, evaluate
% c = sum_i cross(r1(:,i),r2(:,i),1) and its derivative
n_pts = size(r1,2);
size_r1 = size(r1);
size_r2 = size(r2);
if(length(size_r1) ~= 2 || length(size_r2) ~= 2 || size_r1(1) ~= 3 || size_r2(1) ~= 3 || size_r2(2) ~= n_pts)
  error('Drake:crossSum:Incorrect input size')
end
c = sum(cross(r1,r2,1),2);
if(nargout>1)
  row_idx = reshape(bsxfun(@times,[2;3;1;3;1;2],ones(1,n_pts)),[],1);
  col_idx = reshape(bsxfun(@plus,[1;1;2;2;3;3],3*(0:n_pts-1)),[],1);
  dc = sparse([row_idx;row_idx],[col_idx;col_idx+3*n_pts],[reshape([-r2(3,:);r2(2,:);r2(3,:);-r2(1,:);-r2(2,:);r2(1,:)],[],1);reshape([r1(3,:);-r1(2,:);-r1(3,:);r1(1,:);r1(2,:);-r1(1,:)],[],1)],3,6*n_pts);
end
end