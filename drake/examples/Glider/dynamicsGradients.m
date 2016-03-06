function [df] = dynamicsGradients(obj,t,x,u,order)

if (nargin<4) order=1; end
if (order<1) error(' order must be >= 1'); end

Sw=obj.S_w;
Se=obj.S_e;
l=obj.l_h;
lw=obj.l_w;
le=obj.l_e;

m=obj.m;
g=obj.g;
rho=obj.rho;
I=obj.I;

%u=ppval(t,utape);

q3 = x(3);
q4 = x(4);
qdot1 = x(5);
qdot2 = x(6);
qdot3 = x(7);
qdot4 = u(1);

c3=cos(q3);
c34=cos(q3+q4);
s3=sin(q3);
s34=sin(q3+q4);
s4=sin(q4);
c4=cos(q4);

xwdot = qdot1 - lw*qdot3*sin(q3);
zwdot=  qdot2 + lw*qdot3*cos(q3);
alpha_w = q3 - atan2(zwdot,xwdot);
Fw = rho*Sw*sin(alpha_w)*(zwdot*zwdot+xwdot*xwdot);

xedot = qdot1 + l*qdot3*sin(q3) + le*(qdot3+qdot4)*sin(q3+q4); 
zedot = qdot2 - l*qdot3*cos(q3) - le*(qdot3+qdot4)*cos(q3+q4);
alpha_e = q3+q4-atan2(zedot,xedot);
Fe = rho*Se*sin(alpha_e)*(zedot*zedot+xedot*xedot);

dxwdot_dq3=-lw*qdot3*cos(q3);

dxwdot_dqdot3=-lw*sin(q3);

dzwdot_dq3=-lw*qdot3*sin(q3);

dzwdot_dqdot3=lw*cos(q3);

U=1/(1+(zwdot/xwdot)*(zwdot/xwdot));

dalpha_w_dq3=1-U*(xwdot*dzwdot_dq3-zwdot*dxwdot_dq3)/(xwdot*xwdot);

dalpha_w_dqdot1=-U*(-zwdot)/(xwdot*xwdot);
dalpha_w_dqdot2=-U*(xwdot)/(xwdot*xwdot);
dalpha_w_dqdot3=-U*(xwdot*dzwdot_dqdot3-zwdot*dxwdot_dqdot3)/(xwdot*xwdot);

dxedot_dq3=l*qdot3*c3+le*(qdot3+qdot4)*c34;
dxedot_dq4=le*(qdot3+qdot4)*c34;

dxedot_dqdot3=l*sin(q3)+le*s34;
dxedot_dqdot4=le*s34;

dzedot_dq3=l*qdot3*sin(q3)+le*(qdot3+qdot4)*s34;
dzedot_dq4=le*(qdot3+qdot4)*s34;

dzedot_dqdot3=-l*c3-le*c34;
dzedot_dqdot4=-le*c34;

U=1/(1+(zedot/xedot)*(zedot/xedot));

dalpha_e_dq3=1-U*(xedot*dzedot_dq3-zedot*dxedot_dq3)/(xedot*xedot);
dalpha_e_dq4=1-U*(xedot*dzedot_dq4-zedot*dxedot_dq4)/(xedot*xedot);
dalpha_e_dqdot1=-U*(-zedot)/(xedot*xedot);
dalpha_e_dqdot2=-U*(xedot)/(xedot*xedot);
dalpha_e_dqdot3=-U*(xedot*dzedot_dqdot3-zedot*dxedot_dqdot3)/(xedot*xedot);
dalpha_e_dqdot4=-U*(xedot*dzedot_dqdot4-zedot*dxedot_dqdot4)/(xedot*xedot);

W=rho*Sw;
U=cos(alpha_w)*zwdot*zwdot;
P=sin(alpha_w)*2*zwdot;
Z=cos(alpha_w)*xwdot*xwdot;
Y=sin(alpha_w)*2*xwdot;

dFw_dq3=W*(U*dalpha_w_dq3+P*dzwdot_dq3+Z*dalpha_w_dq3+Y*dxwdot_dq3);

dFw_dqdot1=W*(U*dalpha_w_dqdot1+Z*dalpha_w_dqdot1+Y);
dFw_dqdot2=W*(U*dalpha_w_dqdot2+P+Z*dalpha_w_dqdot2);
dFw_dqdot3=W*(U*dalpha_w_dqdot3+P*dzwdot_dqdot3+Z*dalpha_w_dqdot3+Y*dxwdot_dqdot3);

W=rho*Se;
U=cos(alpha_e)*zedot*zedot;
P=sin(alpha_e)*2*zedot;
Z=cos(alpha_e)*xedot*xedot;
Y=sin(alpha_e)*2*xedot;

dFe_dq3=W*(U*dalpha_e_dq3+P*dzedot_dq3+Z*dalpha_e_dq3+Y*dxedot_dq3);
dFe_dq4=W*(U*dalpha_e_dq4+P*dzedot_dq4+Z*dalpha_e_dq4+Y*dxedot_dq4);
dFe_dqdot1=W*(U*dalpha_e_dqdot1+Z*dalpha_e_dqdot1+Y);
dFe_dqdot2=W*(U*dalpha_e_dqdot2+P+Z*dalpha_e_dqdot2);
dFe_dqdot3=W*(U*dalpha_e_dqdot3+P*dzedot_dqdot3+Z*dalpha_e_dqdot3+Y*dxedot_dqdot3);
dFe_dqdot4=W*(U*dalpha_e_dqdot4+P*dzedot_dqdot4+Z*dalpha_e_dqdot4+Y*dxedot_dqdot4);


df5_dq3=(-(dFw_dq3*s3+Fw*c3+dFe_dq3*s34+Fe*c34))/m;
df5_dq4=(-(dFe_dq4*s34+Fe*c34))/m;
df5_dqdot1=(-(dFw_dqdot1*s3+dFe_dqdot1*s34))/m;
df5_dqdot2=(-(dFw_dqdot2*s3+dFe_dqdot2*s34))/m;
df5_dqdot3=-(dFw_dqdot3*s3+dFe_dqdot3*s34)/m;
df5_dqdot4=-(dFe_dqdot4*s34)/m;

df6_dq3=(dFw_dq3*c3-Fw*s3+dFe_dq3*c34-Fe*s34)/m;
df6_dq4=(dFe_dq4*c34-Fe*s34)/m;
df6_dqdot1=(dFw_dqdot1*c3+dFe_dqdot1*c34)/m;
df6_dqdot2=(dFw_dqdot2*c3+dFe_dqdot2*c34)/m;
df6_dqdot3=(dFw_dqdot3*c3+dFe_dqdot3*c34)/m;
df6_dqdot4=(dFe_dqdot4*c34)/m;

df7_dq3=(dFw_dq3*lw-dFe_dq3*(l*c4+le))/I;
df7_dq4=(-dFe_dq4*(l*c4+le)+Fe*(l*s4))/I;
df7_dqdot1=(dFw_dqdot1*lw-dFe_dqdot1*(l*c4+le))/I;
df7_dqdot2=(dFw_dqdot2*lw-dFe_dqdot2*(l*c4+le))/I;
df7_dqdot3=(dFw_dqdot3*lw-dFe_dqdot3*(l*c4+le))/I;
df7_dqdot4=(-dFe_dqdot4*(l*c4+le))/I; 

dfdx(5,1)=0;
dfdx(5,2)=0;    
dfdx(5,3)=df5_dq3;
dfdx(5,4)=df5_dq4;
dfdx(5,5)=df5_dqdot1;
dfdx(5,6)=df5_dqdot2;
dfdx(5,7)=df5_dqdot3;

dfdx(6,1)=0;
dfdx(6,2)=0;
dfdx(6,3)=df6_dq3;
dfdx(6,4)=df6_dq4;
dfdx(6,5)=df6_dqdot1;
dfdx(6,6)=df6_dqdot2;
dfdx(6,7)=df6_dqdot3;

dfdx(7,1)=0;
dfdx(7,2)=0;
dfdx(7,3)=df7_dq3;
dfdx(7,4)=df7_dq4;
dfdx(7,5)=df7_dqdot1;
dfdx(7,6)=df7_dqdot2;
dfdx(7,7)=df7_dqdot3;

dfdx(1,1)=0;
dfdx(1,2)=0;
dfdx(1,3)=0;
dfdx(1,4)=0;
dfdx(1,5)=1;
dfdx(1,6)=0;
dfdx(1,7)=0;


dfdx(2,1)=0;
dfdx(2,2)=0;
dfdx(2,3)=0;
dfdx(2,4)=0;
dfdx(2,5)=0;
dfdx(2,6)=1;
dfdx(2,7)=0;

dfdx(3,1)=0;
dfdx(3,2)=0;
dfdx(3,3)=0;
dfdx(3,4)=0;
dfdx(3,5)=0;
dfdx(3,6)=0;
dfdx(3,7)=1;

dfdx(4,1)=0;
dfdx(4,2)=0;
dfdx(4,3)=0;
dfdx(4,4)=0;
dfdx(4,5)=0;
dfdx(4,6)=0;
dfdx(4,7)=0;

dfdu(1)=0;
dfdu(2)=0;
dfdu(3)=0;
dfdu(4)=1;
dfdu(5)=df5_dqdot4;
dfdu(6)=df6_dqdot4;
dfdu(7)=df7_dqdot4;

dfdu=dfdu';

dfdt=dfdu*0;

df=[dfdt dfdx dfdu];

df=sparse(df);

end

% NOTEST
