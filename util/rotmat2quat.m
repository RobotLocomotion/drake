function [q,dq]=rotmat2quat(M)
% convert rotation matrix to quaternion, based on 
% http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
% Pay close attention that there are two quaternions for the same rotation
% matrix!!!, namely quat2rotmat(q) = quat2rotmat(-q).
traceM = 1+trace(M);
mat_ind = reshape((1:9),3,3);
if(traceM>eps)
    w = sqrt(traceM)/2;
    w4 = w*4;
    x = (M(3,2)-M(2,3))/w4;
    y = (M(1,3)-M(3,1))/w4;
    z = (M(2,1)-M(1,2))/w4;
    if(nargout>1)
      dwdM11 = 1/(8*w);
      dwdM22 = 1/(8*w);
      dwdM33 = 1/(8*w);
      dwdM = zeros(1,9);
      dwdM(1) = dwdM11;
      dwdM(5) = dwdM22;
      dwdM(9) = dwdM33;
      dxdM_nom = zeros(1,9);
      dxdM_nom(mat_ind(3,2)) = 1;
      dxdM_nom(mat_ind(2,3)) = -1;
      dxdM = 1/4*(dxdM_nom*w-(M(3,2)-M(2,3))*dwdM)/w^2;
      dydM_nom = zeros(1,9);
      dydM_nom(mat_ind(1,3)) = 1;
      dydM_nom(mat_ind(3,1)) = -1;
      dydM = 1/4*(dydM_nom*w-(M(1,3)-M(3,1))*dwdM)/w^2;
      dzdM_nom = zeros(1,9);
      dzdM_nom(mat_ind(2,1)) = 1;
      dzdM_nom(mat_ind(1,2)) = -1;
      dzdM = 1/4*(dzdM_nom*w-(M(2,1)-M(1,2))*dwdM)/w^2;
      dq = [dwdM;dxdM;dydM;dzdM];
    end
elseif(M(1,1)>M(2,2)&&M(1,1)>M(3,3))
    s = 2*sqrt(1+M(1,1)-M(2,2)-M(3,3));
    w = (M(3,2)-M(2,3))/s;
    x = 0.25*s;
    y = (M(1,2)+M(2,1))/s;
    z = (M(1,3)+M(3,1))/s;
    if(nargout>1)
      dsdM11 = 1/(8*s);
      dsdM22 = -1/(8*s);
      dsdM33 = -1/(8*s);
      dsdM = zeros(1,9);
      dsdM(1) = dsdM11;
      dsdM(5) = dsdM22;
      dsdM(9) = dsdM33;
      dwdM_nom = zeros(1,9);
      dwdM_nom(mat_ind(3,2)) = 1;
      dwdM_nom(mat_ind(2,3)) = -1;
      dwdM = (dwdM_nom*s-(M(3,2)-M(2,3))*dsdM)/s^2;
      dxdM = 0.25*dsdM;
      dydM_nom = zeros(1,9);
      dydM_nom(mat_ind(1,2)) = 1;
      dydM_nom(mat_ind(2,1)) = 1;
      dydM = (dydM_nom*s-(M(1,2)+M(2,1))*dsdM)/s^2;
      dzdM_nom = zeros(1,9);
      dzdM_nom(mat_ind(1,3)) = 1;
      dzdM_nom(mat_ind(3,1)) = 1;
      dzdM = (dzdM_nom*s-(M(1,3)+M(3,1))*dsdM)/s^2;

    end
elseif(M(2,2)>M(3,3))
    s = 2*(sqrt(1+M(2,2)-M(1,1)-M(3,3)));
    w = (M(1,3)-M(3,1))/s;
    x = (M(1,2)+M(2,1))/s;
    y = 0.25*s;
    z = (M(2,3)+M(3,2))/s;
    if(nargout>1)
      dsdM11 = -1/(8*s);
      dsdM22 = 1/(8*s);
      dsdM33 = -1/(8*s);
      dsdM = zeros(1,9);
      dsdM(1) = dsdM11;
      dsdM(5) = dsdM22;
      dsdM(9) = dsdM33;
      dwdM_nom = zeros(1,9);
      dwdM_nom(mat_ind(1,3)) = 1;
      dwdM_nom(mat_ind(3,1)) = -1;
      dwdM = (dwdM_nom*s-(M(1,3)-M(3,1))*dsdM)/s^2;
      dxdM_nom = zeros(1,9);
      dxdM_nom(mat_ind(1,2)) = 1;
      dxdM_nom(mat_ind(2,1)) = 1;
      dxdM = (dxdM_nom*s-(M(1,2)+M(2,1))*dsdM)/s^2;
      dydM = 0.25*dsdM;
      dzdM_nom = zeros(1,9);
      dzdM_nom(mat_ind(2,3)) = 1;
      dzdM_nom(mat_ind(3,2)) = 1;
      dzdM = (dzdM_nom*s-(M(2,3)+M(3,2))*dsdM)/s^2;
    end
else
    s = 2*(sqrt(1+M(3,3)-M(2,2)-M(1,1)));
    w = (M(2,1)-M(1,2))/s;
    x = (M(1,3)+M(3,1))/s;
    y = (M(2,3)+M(3,2))/s;
    z = 0.25*s;
    if(nargout>1)
      dsdM11 = -1/(8*s);
      dsdM22 = -1/(8*s);
      dsdM33 =  1/(8*s);
      dsdM = zeros(1,9);
      dsdM(1) = dsdM11;
      dsdM(5) = dsdM22;
      dsdM(9) = dsdM33;
      dwdM_nom = zeros(1,9);
      dwdM_nom(mat_ind(2,1)) = 1;
      dwdM_nom(mat_ind(1,2)) = -1;
      dwdM = (dwdM_nom*s-(M(2,1)-M(1,2))*dsdM)/s^2;
      dxdM_nom = zeros(1,9);
      dxdM_nom(mat_ind(1,3)) = 1;
      dxdM_nom(mat_ind(3,1)) = 1;
      dxdM = (dxdM_nom*s-(M(1,3)+M(3,1))*dsdM)/s^2;
      dydM_nom = zeros(1,9);
      dydM_nom(mat_ind(2,3)) = 1;
      dydM_nom(mat_ind(3,2)) = 1;
      dydM = (dydM_nom*s-(M(2,3)+M(3,2))*dsdM)/s^2;
      dzdM = 0.25*dsdM;
    end
end
q = [w;x;y;z];
if(nargout>1)
  dq = [dwdM;dxdM;dydM;dzdM];
end
end