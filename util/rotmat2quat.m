function q=rotmat2quat(M)
% convert rotation matrix to quaternion, based on 
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
% Pay close attention that there are two quaternions for the same rotation
% matrix!!!, namely quat2rotmat(q) = quat2rotmat(-q).

[val,ind] = max([1 1 1; 1 -1 -1; -1 1 -1; -1 -1 1]*diag(M));

switch(ind)
  case 1  % val = trace(M)
    w = sqrt(1+val)/2;
    w4 = w*4;
    x = (M(3,2)-M(2,3))/w4;
    y = (M(1,3)-M(3,1))/w4;
    z = (M(2,1)-M(1,2))/w4;
  case 2 % val = M(1,1) - M(2,2) - M(3,3)
    s = 2*sqrt(1+val);
    w = (M(3,2)-M(2,3))/s;
    x = 0.25*s;
    y = (M(1,2)+M(2,1))/s;
    z = (M(1,3)+M(3,1))/s;
  case 3 % val = M(2,2) - M(1,1) - M(3,3)
    s = 2*(sqrt(1+val));
    w = (M(1,3)-M(3,1))/s;
    x = (M(1,2)+M(2,1))/s;
    y = 0.25*s;
    z = (M(2,3)+M(3,2))/s;
  otherwise % val = M(3,3) - M(2,2) - M(1,1) 
    s = 2*(sqrt(1+val));
    w = (M(2,1)-M(1,2))/s;
    x = (M(1,3)+M(3,1))/s;
    y = (M(2,3)+M(3,2))/s;
    z = 0.25*s;
end

q = [w;x;y;z];

end