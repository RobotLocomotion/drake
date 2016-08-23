function R = rotmatFromQuatBilinear(Quat)
% Quat is the bilinear matrix quat*quat', where quat is the quaternion of the
% rotation
if(any(size(Quat) ~= [4,4]))
  error('Quat should be a 4 x 4 matrix');
end
R = [Quat(2,2)+Quat(1,1)-Quat(4,4)-Quat(3,3)   2*Quat(2,3)-2*Quat(1,4)   2*Quat(2,4)+2*Quat(1,3);...
       2*Quat(2,3)+2*Quat(1,4)   Quat(3,3)+Quat(1,1)-Quat(2,2)-Quat(4,4)   2*Quat(3,4)-2*Quat(1,2);...
       2*Quat(2,4)-2*Quat(1,3)   2*Quat(3,4)+2*Quat(1,2)   Quat(4,4)+Quat(1,1)-Quat(3,3)-Quat(2,2)];
end