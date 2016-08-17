function [f,df] = squaredEffort(R,h,x,u)
  f = h*(u'*R*u);
  df = [(u'*R*u),zeros(1,size(x,1)),2*h*u'*R];
end
