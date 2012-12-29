function [com,J,dJ] = getCOM(model,q)

% return total mass and center of mass for the entire model
m = 0;
com = zeros(model.D,1);
if (nargout>1)
  J = zeros(model.D,length(q));
end

use_mex = false;
if (model.mex_model_ptr && isnumeric(q))
  use_mex = true;
end
planar=(model.D==2);

doKinematics(model,q,nargout>2,use_mex);

dJ = zeros(model.D,length(q)^2);
for i=1:length(model.body)
  bm = model.body(i).mass;
  bc = model.body(i).com;
  if (bm>0)
    if (nargout>2)
      [bc,bJ,bdJ] = forwardKin(model,i,bc,use_mex);
      J = (m*J + bm*bJ)/(m+bm);
      dJ = (m*dJ + bm*bdJ)/(m+bm);
    elseif (nargout>1)
      [bc,bJ] = forwardKin(model,i,bc,use_mex);
      J = (m*J + bm*bJ)/(m+bm);
    else
      bc = forwardKin(model,i,bc,use_mex);
    end
    com = (m*com + bm*bc)/(m+bm);
    m = m + bm;
  end
end

end