function [r,dr] = quatDiff(q1,q2)
% function [r,dr] = quatDiff(q1,q2)
%
% Compute the quaternion (and gradient) of the relative transform
% between two quaternions.
%
% r = conj(q1)*q2

% Standard multiplication matrices, for reference
%   W = diag([1;-1;-1;-1]);
%   X = [0 1 0 0;1 0 0 0;0 0 0 1;0 0 -1 0];
%   Y = [0 0 1 0;0 0 0 -1; 1 0 0 0; 0 1 0 0];
%   Z = [0 0 0 1; 0 0 1 0; 0 -1 0 0;  1 0 0 0];

% Since we're taking the difference, we want conj(q1)*q2
% So swap the signs of everything outside of the first row

sizecheck(q2,[4,1]);
sizecheck(q2,[4,1]);

W = diag([1;1;1;1]);
X = [0 1 0 0;-1 0 0 0;0 0 0 -1;0 0 1 0];
Y = [0 0 1 0;0 0 0 1; -1 0 0 0; 0 -1 0 0];
Z = [0 0 0 1; 0 0 -1 0; 0 1 0 0;-1 0 0 0];

r = [q1'*W*q2;
  q1'*X*q2;
  q1'*Y*q2;
  q1'*Z*q2];

dr = [q2'*W' q1'*W; q2'*X' q1'*X; q2'*Y' q1'*Y; q2'*Z' q1'*Z];
end