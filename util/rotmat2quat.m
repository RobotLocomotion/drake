function q=rotmat2quat(M)
% convert rotation matrix to quaternion, based on 
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
% Pay close attention that there are two quaternions for the same rotation
% matrix!!!, namely quat2rotmat(q) = quat2rotmat(-q).
traceM = 1+trace(M);
if(traceM>eps)
    w = sqrt(traceM)/2;
    w4 = w*4;
    x = (M(3,2)-M(2,3))/w4;
    y = (M(1,3)-M(3,1))/w4;
    z = (M(2,1)-M(1,2))/w4;
else
    if(M(1,1)>M(2,2)&&M(1,1)>M(3,3))
        s = 2*sqrt(1+M(1,1)-M(2,2)-M(3,3));
        w = (M(3,2)-M(2,3))/s;
        x = 0.25*s;
        y = (M(1,2)+M(2,1))/s;
        z = (M(1,3)+M(3,1))/s;
    elseif(M(2,2)>M(3,3))
        s = 2*(sqrt(1+M(2,2)-M(1,1)-M(3,3)));
        w = (M(1,3)-M(3,1))/s;
        x = (M(1,2)+M(2,1))/s;
        y = 0.25*s;
        z = (M(2,3)+M(3,2))/s;
    else
        s = 2*(sqrt(1+M(3,3)-M(2,2)-M(1,1)));
        w = (M(2,1)-M(1,2))/s;
        x = (M(1,3)+M(3,1))/s;
        y = (M(2,3)+M(3,2))/s;
        z = 0.25*s;
    end
end
q = [w;x;y;z];
end