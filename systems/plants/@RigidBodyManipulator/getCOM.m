function [com,J,dJ] = getCOM(model,q)

% return total mass and center of mass for the entire model
m = 0;
d = length(model.gravity);  % so it works for both 2D and 3D
com = zeros(d,1);
if (nargout>1)
  J = zeros(d,length(q));
end

kinsol = doKinematics(model,q,nargout>2);

dJ = zeros(d,length(q)^2);
for i=1:length(model.body)
  bm = model.body(i).mass;
  bc = model.body(i).com;
  if (bm>0)
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