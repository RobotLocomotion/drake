function [w2_unwrap,dw2_unwrap_dw2] = unwrapExpmap(w1,w2)
% unwrap the exponential map w2, such that w2 is close to w1
% @param w1   A 3 x 1 vector, the exponential map
% @param w2   A 3 x 1 vector, the exponential map
% @retval dw2_unwrap_dw2  % The gradient of w2_unwrap w.r.t w2
assert(all(size(w2) == [3,1]) && all(size(w1) == [3,1]));
[w2_flip,dw2_flip_dw2] = flipExpmap(w2);
% compare the distance of w2_flip to w1 with w2 to w1
expmap_dist = sum([w2-w1 w2_flip-w1].^2);
if(expmap_dist(1)>expmap_dist(2))
  w2_unwrap = w2_flip;
  dw2_unwrap_dw2 = dw2_flip_dw2;
else
  w2_unwrap = w2;
  dw2_unwrap_dw2 = eye(3);
end
end