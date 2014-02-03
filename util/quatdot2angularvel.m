function omega = quatdot2angularvel(q, qd)
%*   Convert derivative of quaternion to angular velocity in base frame
% from Schwab, Arend L. "Quaternions, finite rotation and euler
% parameters." (self-published)
% http://bicycle.tudelft.nl/schwab/Publications/quaternion.pdf

omega = quatdot2angularvelMatrix(q) * qd;

end