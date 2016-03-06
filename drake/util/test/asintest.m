function asintest
% tests my implementation of asin in TaylorVar.  

checkDependency('spotless');

order=7;
x0=.5;
tx=TaylorVar.init(x0,order);
a=asin(tx);
px=msspoly('x',1);
pa=getmsspoly(a,px-x0);

xs=linspace(0,1,21);
clf
plot(xs,asin(xs),xs,double(msubs(pa,px,xs)));
legend('asin','taylor');

