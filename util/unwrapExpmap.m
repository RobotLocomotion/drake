function [w2_unwrap,dw2_unwrap_dw2] = unwrapExpmap(w1,w2)
% unwrap the exponential map w2, such that w2 is close to w1
% @param w1   A 3 x 1 vector, the exponential map
% @param w2   A 3 x 1 vector, the exponential map
% @retval flip_flag    A boolean, if true, then the exponential map is
% flipped
assert(all(size(w2) == [3,1]) && all(size(w1) == [3,1]));
if(norm(w2-w1)>2*pi)
  [w2_unwrap,dw2_unwrap_dw2] = flipExpmap(w2);
else
  w2_unwrap = w2;
  dw2_unwrap_dw2 = eye(3);
end
end