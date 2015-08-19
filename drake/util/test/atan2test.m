function atan2test
% tests my implementation of atan2 in TaylorVar.  
% it's a little harder than most, since matlab symbolic toolbox doesn't 
% implement atan2 (which is very annoying).

options.grad_method={'user','taylorvar'};
[f,df,df2]=geval(@myatan2,randn,randn,options);

return

% graphical version
order=7;
x0=[.5;.5];
tx=TaylorVar.init(x0,order);
a=atan2(tx(1),tx(2));
px=msspoly('x',2);
pa=getmsspoly(a,px-x0);

[xs,ys]=meshgrid(linspace(0,1,21),linspace(0,1,21));

clf;
mesh(xs,ys,atan2(xs,ys));
hold on;
mesh(xs,ys,reshape(double(msubs(pa,px,[xs(:)';ys(:)'])),size(xs)));




end


function df = datan2(y,x);
  df(1,1) = x./(x.^2+y.^2);
  df(1,2) = -y./(x.^2+y.^2);
end


function [f,df,df2]=myatan2(y,x)
  f=atan2(y,x);
  if (nargout>1)
    df=datan2(y,x);
    df2=[ -(2*x*y)/(x^2 + y^2)^2, (2*y^2)/(x^2 + y^2)^2 - 1/(x^2 + y^2), 1/(x^2 + y^2) - (2*x^2)/(x^2 + y^2)^2, (2*x*y)/(x^2 + y^2)^2];
    % df2 derived using:
    %  syms y x;
    %  df=datan2(y,x);
    %  reshape(jacobian(reshape(df,2,1),[y,x]),1,4)
  end
end
