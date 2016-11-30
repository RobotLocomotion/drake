function [e,de] = quatDiffAxisInvar(q1,q2,u)
% function [e,de] = quatDiffAxisInvar(q1,q2,u)
%
% computes the error term, and gradient, of the difference between
% two quaternions, where the relative transform is invariant to rotation
% about a specific axis
%
% e = 0 iff quatDiff(q1,q2) is a rotation about the axis u
%
% Specifically, this transforms quatDiff(q1,q2) to axis-angle form
% as a rotation of theta about unit vector w, and returns
%
% e = 2*sin(theta)^2*(u'w - 1)

sizecheck(q1,[4,1]);
sizecheck(q2,[4,1]);
sizecheck(u,[3,1]);

[r,dr] = quatDiff(q1,q2);

e = -2 + 2*r(1)^2 + 2*(u'*r(2:4))^2;

de = [4*r(1)*dr(1,:) + 4*(u'*r(2:4))*u'*dr(2:4,:) 4*(u'*r(2:4))*r(2:4)'];
end