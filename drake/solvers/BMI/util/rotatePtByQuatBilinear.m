function pos = rotatePtByQuatBilinear(Quat,pt)
% Given points 'pt', return the position of the points after they are rotated
% by a quaternion, namely [0;pos] = quat*[0;pt]*quat', where '*' is the
% Hamiltonian product for quaternions, quat' is the conjugate of quaternion
% @param Quat A 4 x 4 matrix, Quat is the bilinear product quat*quat',
% where quat is the quaternion representing the rotation
% @param pt   A 3 x N matrix, the position of the points before the rotation
% @param pos  A 3 x N matrix, the position of the points after the rotation
if(any(size(Quat) ~= [4,4]) || size(pt,1) ~= 3)
  error('Quat should be a 4 x 4 matrix, pt should be a 3 x 1 vector');
end
pos = [Quat(2,2)+Quat(1,1)-Quat(4,4)-Quat(3,3)   2*Quat(2,3)-2*Quat(1,4)   2*Quat(2,4)+2*Quat(1,3);...
       2*Quat(2,3)+2*Quat(1,4)   Quat(3,3)+Quat(1,1)-Quat(2,2)-Quat(4,4)   2*Quat(3,4)-2*Quat(1,2);...
       2*Quat(2,4)-2*Quat(1,3)   2*Quat(3,4)+2*Quat(1,2)   Quat(4,4)+Quat(1,1)-Quat(3,3)-Quat(2,2)]*pt;
end