function bodyKinTest

oldpath=addpath(fullfile(pwd,'..'));

N = 1+randi(10);
r = PlanarNLink(N);

kinsol = doKinematics(r,randn(r.getNumDOF,1));

pts = randn(2,20);
x = forwardKin(r,kinsol,N+1,pts);
pts2 = bodyKin(r,kinsol,N+1,x);
valuecheck(pts2,pts);

path(oldpath);

end