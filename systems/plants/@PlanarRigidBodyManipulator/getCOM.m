function [com,J,dJ] = getCOM(model,kinsol)

if ~isstruct(kinsol)  
  % treat input as getCOM(model,q)
  kinsol = doKinematics(model,kinsol,nargin<2);
end

% todo: implement getCOM in mex (like in RigidBodyManipulator)

nq=getNumDOF(model);

% return center of mass for the entire model
m=0;
d = 2;
com = zeros(d,1);
if (nargout>1)
  J = zeros(d,nq);
end

dJ = zeros(d,nq^2);
for i=1:length(model.body)
  bm = model.body(i).mass;
  if (bm>0)
    bc = model.body(i).com;
    if (nargout>2)
      [bc,bJ,bdJ] = forwardKin(model,kinsol,i,bc);
      J = (m*J + bm*bJ)/(m+bm);
      dJ = (m*dJ + bm*bdJ)/(m+bm);
    elseif (nargout>1)
      [bc,bJ] = forwardKin(model,kinsol,i,bc);
      J = (m*J + bm*bJ)/(m+bm);
    else
      bc = forwardKin(model,kinsol,i,bc);
    end
    com = (m*com + bm*bc)/(m+bm);
    m = m + bm;
  end
end

end