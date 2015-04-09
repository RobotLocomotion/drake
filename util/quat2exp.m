function w = quat2exp(q)
% Given quaternion q, return the exponential representation of the rotation
% matrix, such that axis_angle_i = quat2axis(q(:,i)) and w(:,i) =
% axis(i)*angle(i)
% @param q   A 4 x N matrix, q(:,i) is a quaternion
% @retval w   A 3 x N matrix, w(:,i) is exponential representation of q(:,i)
assert(isa(q,'numeric'));
assert(size(q,1) == 4);
N = size(q,2);
q = q./bsxfun(@times,ones(4,1),sqrt(sum(q.^2,1)));
is_degenerate = abs(q(1,:).^2-ones(1,N))<eps;
s = zeros(1,N);
s(~is_degenerate) = 2*acos(q(1,~is_degenerate))./sqrt(1-q(1,~is_degenerate).^2);
s(is_degenerate) = 2;
w = q(2:4,:).*bsxfun(@times,ones(3,1),s);
end