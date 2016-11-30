function ret = transformAdjoint(H)
% returns the adjoint to a homogeneous transform, i.e. returns
% [R        0;
%  pHat * R R]
%
% for H =
% [R p;
% [0 1]
%
% where pHat is the skew symmetric matrix corresponding to the cross
% product operation

R = H(1 : 3, 1 : 3);
p = H(1 : 3, 4);
pHat = vectorToSkewSymmetric(p);
ret = [R, zeros(3, 3);
  pHat * R, R];
end