function [wdot,w] = quatdot2expmapdot(q,qdot)
% Given quaternion q and quaternion time derivative qdot, return the time
% derivative of exponential representation
% @param q   A 4 x N matrix, q(:,i) is a quaternion
% @param qdot  A 4 X N matrix, qdot(:,i) is the time derivative of q(:,i)
% @param w   A 3 x N matrix, w(:,i) is the exponential representation for
% q(:,i)
% @retval wdot  A 3 x N matrix, wdot(:,i) is the time derivative of
% exponential representation for q(:,i)
assert(isa(q,'numeric') && isa(qdot,'numeric'));
assert(size(q,1) == 4 && size(qdot,1) == 4);
N = size(q,2);
assert(size(qdot,2) == N);
is_degenerate = abs(q(1,:).^2-ones(1,N))<eps;
q = q./bsxfun(@times,ones(4,1),sqrt(sum(q.^2,1)));
s = zeros(1,N);
s(~is_degenerate) = 2*acos(q(1,~is_degenerate))./sqrt(1-q(1,~is_degenerate).^2);
s(is_degenerate) = 2;
w = q(2:4,:).*bsxfun(@times,ones(3,1),s);
dsdq1 = zeros(1,N);
dsdq1(~is_degenerate) = (-2*sqrt(1-q(1,~is_degenerate).^2)+2*acos(q(1,~is_degenerate)).*q(1,~is_degenerate))./((1-q(1,~is_degenerate).^2).^(1.5));
dsdq1(is_degenerate) = 0;
row_idx = reshape(bsxfun(@plus,[1;2;3;1;2;3],3*(0:N-1)),[],1);
col_idx = reshape(bsxfun(@plus,[1;1;1;2;3;4],4*(0:N-1)),[],1);
val = reshape([q(2,:).*dsdq1;q(3,:).*dsdq1;q(4,:).*dsdq1;s;s;s],[],1);
wdot = reshape(sparse(row_idx,col_idx,val,3*N,4*N)*qdot(:),3,N);
end