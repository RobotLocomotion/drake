function bodyKinTest

oldpath=addpath('..');

N = 1+ceil(rand);
r = PlanarNLink(N);

pts = randn(2,20);

use_mex = false;
kinsol = doKinematics(r,randn(r.getNumDOF,1),true,use_mex);
x = forwardKin(r,kinsol,N,pts);
pts2 = bodyKin(r,kinsol,N,x);
valuecheck(pts,pts2);

path(oldpath);

end