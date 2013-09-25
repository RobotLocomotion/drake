function [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,userfun,wrapupfun,iname,oname]=...
    rtrlSNOPTtranscription(sys,costFun,finalCostFun,x0,uTraj0,con,options)

%The options are left vacant. To-do: to implement the gradient test.
nU=sys.getNumInputs();
nXc=sys.getNumContStates();
t=uTraj0.getBreaks();
tspan=t(end)-t(1);
u=uTraj0.eval(t);
nT = length(t);
t0=t(1);
if(~isfield(con,'u'))
    con.u.ub=sys.umax;
    con.u.lb=sys.umin;
else
if(~isfield(con.u,'ub'))
    con.u.ub=sys.umax;
end
if(~isfield(con.u,'lb'))
    con.u.lb=sys.umin;
end
end
if(sys.getNumDiscStates()>0)
    error('not implemented yet (but should be very straightforward)');
end

ts = sys.getSampleTime();
if (size(ts,2)>1 || any(ts~=0))
  error('not implemented yet (but should be very straightforward)');
end

if(~isfield(con,'nKnotsInSubTraj'))
    con.nKnotsInSubTraj=10;
end

if(~isfield(con,'tf'))
    con.tf.ub=tspan;
    con.tf.lb=tspan;
end

if(~isfield(con,'x'))
    con.x.ub=inf(nXc,1);
    con.x.lb=-inf(nXc,1);
end
nSubTraj=ceil((nT-1)/con.nKnotsInSubTraj);
nKnotsInSubTraj=zeros(1,nSubTraj);
tSubTrajStart=t(mod((1:(length(t)-1)),con.nKnotsInSubTraj)==1);
sys_input=cascade(uTraj0,sys);
xTraj0=simulate(sys_input,t,x0);
xSubTrajStart=xTraj0.eval(tSubTrajStart);
w0=[tspan;xSubTrajStart(:);u(:)];
whigh=[con.tf.ub;x0;repmat(con.x.ub,nSubTraj-1,1);repmat(con.u.ub,nT,1)];
wlow=[con.tf.lb;x0;repmat(con.x.lb,nSubTraj-1,1);repmat(con.u.lb,nT,1)];

iGfun=ones(1+nSubTraj*nXc+nU*nT,1);
jGvar=(1:(1+nSubTraj*nXc+nU*nT))';
Fhigh=inf;
Flow=-inf;
for i=1:nSubTraj-1
    nKnotsInSubTraj(i)=con.nKnotsInSubTraj;
    iGfun=[iGfun;max(iGfun)+(1:nXc)'; max(iGfun)+reshape((1:nXc)'*ones(1,nXc),[],1);max(iGfun)+(1:nXc)';max(iGfun)+reshape((1:nXc)'*ones(1,nU*(nKnotsInSubTraj(i)+1)),[],1)];
    jGvar=[jGvar;ones(nXc,1);1+(i-1)*nXc+reshape(ones(nXc,1)*(1:nXc),[],1);1+i*nXc+(1:nXc)';1+nSubTraj*nXc+sum(nKnotsInSubTraj(1:i))-nKnotsInSubTraj(i)+reshape(ones(nXc,1)*(1:nU*(nKnotsInSubTraj(i)+1)),[],1)];
    Fhigh=[Fhigh;zeros(nXc,1)];
    Flow=[Flow;zeros(nXc,1)];
end
nKnotsInSubTraj(end)=nT-sum(nKnotsInSubTraj);
if(isfield(con,'xf'))
    if(isfield(con.xf,'ub')||isfield(con.xf,'lb'))
        iGfun=[iGfun;max(iGfun)+(1:nXc)';max(iGfun)+reshape((1:nXc)'*ones(1,nXc),[],1);max(iGfun)+reshape((1:nXc)'*ones(1,nU*nKnotsInSubTraj(end)),[],1)];
        jGvar=[jGvar;ones(nXc,1);1+(nSubTraj-1)*nXc+reshape(ones(nXc,1)*(1:nXc),[],1);1+nSubTraj*nXc+sum(nKnotsInSubTraj(1:(nSubTraj-1)))+reshape(ones(nXc,1)*(1:nU*nKnotsInSubTraj(nSubTraj)),[],1)];
        Fhigh=[Fhigh;con.xf.ub];
        Flow=[Flow;con.xf.lb];
    end
    if(isfield(con.xf,'ceq'))
        c=feval(con.xf.ceq,xTraj0.eval(xTraj0.tspan(end)));
        nC=length(c);
        iGfun=[iGfun;max(iGfun)+(1:nC)';max(iGfun)+reshape((1:nC)'*ones(1,nXc),[],1);max(iGfun)+reshape((1:nC)'*ones(1,nU*nKnotsInSubTraj(end)),[],1)];
        jGvar=[jGvar;ones(nC,1);1+(nSubTraj-1)*nXc+reshape(ones(nC,1)*(1:nXc),[],1);1+nSubTraj*nXc+sum(nKnotsInSubTraj(1:(nSubTraj-1)))+reshape(ones(nC,1)*(1:nU*nKnotsInSubTraj(nSubTraj)),[],1)];
        Fhigh=[Fhigh;zeros(nC,1)];
        Flow=[Flow;zeros(nC,1)];
    end
end
iname={};
oname={};
A=[];iAfun=[];jAvar=[];

userfun=@(w) rtrl_userfun(sys,w,costFun,finalCostFun,nT,nXc,nU,nSubTraj,nKnotsInSubTraj,t0,con,options);
%keyboard
wrapupfun=@(w) rtrl_wrapup(sys,w,nT,nXc,nU,nSubTraj,nKnotsInSubTraj,t0);
end


function [f,G]=rtrl_userfun(sys,w,costFun,finalCostFun,nT,nXc,nU,nSubTraj,nKnotsInSubTraj,t0,con,options)
f=zeros(1+nXc*(nSubTraj-1),1);
G=zeros(1+nXc*nSubTraj+nU*nT+nXc*(1+1+nXc)*(nSubTraj-1)+nXc*nU*(nT-nKnotsInSubTraj(end)+nSubTraj-1),1);
tspan=w(1);
xSubTrajStart=reshape(w(1+(1:nXc*nSubTraj)),nXc,nSubTraj);
u=reshape(w((1+nXc*nSubTraj+1):end),nU,[]);
%uSubTraj=cell(1,nSubTraj);
PsubTraj=cell(1,nSubTraj);
dJdalphaSubTraj=cell(1,nSubTraj);
dJdalpha=zeros(1,1+nXc*nSubTraj+nU*nT);
dt=tspan/(nT-1);
ddtdtf=1/(nT-1);
t=t0:dt:(t0+dt*(nT-1));
J=0;
for i=1:(nSubTraj-1)
    %in each sub trajectory, alpha=[tf;x0;u(in sub trajectory+1)]
    %uSubTraj{i}=u(:,(sum(nKnotsInSubTraj(1:i))-nKnotsInSubTraj(i)+1):sum(nKnotsInSubTraj(1:i)));
    PsubTraj{i}=zeros(nXc,1+nXc+nU*(nKnotsInSubTraj(i)+1));
    PsubTraj{i}(:,1+(1:nXc))=eye(nXc);
    dJdalphaSubTraj{i}=zeros(1,1+nXc+nU*(nKnotsInSubTraj(i)+1));
    xSubTraj=zeros(nXc,nKnotsInSubTraj(i)+1);
    xSubTraj(:,1)=xSubTrajStart(:,i);
    knotsSubTrajStart=sum(nKnotsInSubTraj(1:i))-nKnotsInSubTraj(i);
    for j=1:nKnotsInSubTraj(i)-1
        [xSubTraj(:,j+1),dxndt,dxndx,dxndu1,dxndu2,dxnddt]=...
            rungeKuttaDynamics(sys,t(knotsSubTrajStart+j),xSubTraj(:,j),u(:,knotsSubTrajStart+j),u(:,knotsSubTrajStart+j+1),dt);
        du1dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(i)+1));
        du2dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(i)+1));
        ddtdalphaSubTraj=zeros(1,1+nXc+nU*(nKnotsInSubTraj(i)+1));
        du1dalphaSubTraj(:,1+nXc+(j-1)*nU+(1:nU))=eye(nU);
        du2dalphaSubTraj(:,1+nXc+j*nU+(1:nU))=eye(nU);
        ddtdalphaSubTraj(1)=ddtdtf;
        PsubTraj{i}=dxndx*PsubTraj{i}+dxndu1*du1dalphaSubTraj+dxndu2*du2dalphaSubTraj+dxnddt*ddtdalphaSubTraj;
        [g,dg]=costFun(t(knotsSubTrajStart+j+1),xSubTraj(:,j+1),u(:,knotsSubTrajStart+j+1));
        J=J+g*dt;
        dgdx=dg(:,1+(1:nXc));
        dgdu=dg(:,1+nXc+(1:nU));
        dJdalphaSubTraj{i}=dJdalphaSubTraj{i}+dgdx*PsubTraj{i}*dt+dgdu*du2dalphaSubTraj*dt+g*ddtdalphaSubTraj;
    end
    [xSubTraj(:,nKnotsInSubTraj(i)+1),dxndt,dxndx,dxndu1,dxndu2,dxnddt]=...
        rungeKuttaDynamics(sys,t(knotsSubTrajStart+nKnotsInSubTraj(i)+1),xSubTraj(:,nKnotsInSubTraj(i)),u(:,knotsSubTrajStart+nKnotsInSubTraj(i)),u(:,knotsSubTrajStart+nKnotsInSubTraj(i)+1),dt);
    du1dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(i)+1));
    du2dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(i)+1));
    ddtdalphaSubTraj=zeros(1,1+nXc+nU*(nKnotsInSubTraj(i)+1));
    du1dalphaSubTraj(:,1+nXc+nU*(nKnotsInSubTraj(i)-1)+(1:nU))=eye(nU);
    du2dalphaSubTraj(:,1+nXc+nU*nKnotsInSubTraj(i)+(1:nU))=eye(nU);
    ddtdalphaSubTraj(1)=ddtdtf;
    PsubTraj{i}=dxndx*PsubTraj{i}+dxndu1*du1dalphaSubTraj+dxndu2*du2dalphaSubTraj+dxnddt*ddtdalphaSubTraj;
    [g,dg]=costFun(t(knotsSubTrajStart+nKnotsInSubTraj(i)),xSubTraj(:,nKnotsInSubTraj(i)+1),u(:,knotsSubTrajStart+nKnotsInSubTraj(i)+1));
    J=J+g*dt;
    dgdx=dg(:,1+(1:nXc));
    dgdu=dg(:,1+nXc+(1:nU));
    dJdalphaSubTraj{i}=dJdalphaSubTraj{i}+dgdx*PsubTraj{i}*dt+dgdu*du2dalphaSubTraj*dt+g*ddtdalphaSubTraj;
    f(1+(i-1)*nXc+(1:nXc))=xSubTraj(:,nKnotsInSubTraj(i)+1)-xSubTrajStart(:,i+1);
    dJdalpha(1)=dJdalphaSubTraj{i}(1)+dJdalpha(1);
    dJdalpha(1+(i-1)*nXc+(1:nXc))=dJdalphaSubTraj{i}(1+(1:nXc))+dJdalpha(1+(i-1)*nXc+(1:nXc));
    dJdalpha(1+nXc*nSubTraj+nU*knotsSubTrajStart+(1:nU*(nKnotsInSubTraj(i)+1)))=dJdalpha(1+nXc*nSubTraj+nU*knotsSubTrajStart+(1:nU*(nKnotsInSubTraj(i)+1)))...
        +dJdalphaSubTraj{i}(1+nXc+(1:nU*(nKnotsInSubTraj(i)+1)));
    G(1+nXc*nSubTraj+nT*nU+(i-1)*nXc*(1+1+nXc)+nXc*nU*(knotsSubTrajStart+i-1)+(1:nXc))=PsubTraj{i}(:,1);%dxndtf part
    G(1+nXc*nSubTraj+nT*nU+(i-1)*nXc*(1+1+nXc)+nXc*nU*(knotsSubTrajStart+i-1)+nXc+(1:nXc^2))=reshape(PsubTraj{i}(:,1+(1:nXc)),[],1);
    G(1+nXc*nSubTraj+nT*nU+(i-1)*nXc*(1+1+nXc)+nXc*nU*(knotsSubTrajStart+i-1)+nXc+nXc^2+(1:nXc))=-ones(nXc,1);
    G(1+nXc*nSubTraj+nT*nU+(i-1)*nXc*(1+1+nXc)+nXc*nU*(knotsSubTrajStart+i-1)+nXc+nXc^2+nXc+(1:nXc*nU*(nKnotsInSubTraj(i)+1)))=reshape(PsubTraj{i}(:,1+nXc+(1:nU*(nKnotsInSubTraj(i)+1))),[],1);
end
%The last sub trajectory
PsubTraj{nSubTraj}=zeros(nXc,1+nXc+nU*nKnotsInSubTraj(nSubTraj));
PsubTraj{nSubTraj}(:,1+(1:nXc))=eye(nXc);
dJdalphaSubTraj{nSubTraj}=zeros(1,1+nXc+nU*nKnotsInSubTraj(nSubTraj));
xSubTraj=zeros(nXc,nKnotsInSubTraj(nSubTraj));
xSubTraj(:,1)=xSubTrajStart(:,nSubTraj);
knotsSubTrajStart=sum(nKnotsInSubTraj(1:nSubTraj))-nKnotsInSubTraj(nSubTraj);
for j=1:nKnotsInSubTraj(nSubTraj)-1
    [xSubTraj(:,j+1),dxndt,dxndx,dxndu1,dxndu2,dxnddt]=...
        rungeKuttaDynamics(sys,t(knotsSubTrajStart+j),xSubTraj(:,j),u(:,knotsSubTrajStart+j),u(:,knotsSubTrajStart+j+1),dt);
    du1dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(nSubTraj)));
    du2dalphaSubTraj=zeros(nU,1+nXc+nU*(nKnotsInSubTraj(nSubTraj)));
    ddtdalphaSubTraj=zeros(1,1+nXc+nU*(nKnotsInSubTraj(nSubTraj)));
    du1dalphaSubTraj(:,1+nXc+(j-1)*nU+(1:nU))=eye(nU);
    du2dalphaSubTraj(:,1+nXc+j*nU+(1:nU))=eye(nU);
    ddtdalphaSubTraj(1)=ddtdtf;
    PsubTraj{nSubTraj}=dxndx*PsubTraj{nSubTraj}+dxndu1*du1dalphaSubTraj+dxndu2*du2dalphaSubTraj+dxnddt*ddtdalphaSubTraj;
    [g,dg]=costFun(t(knotsSubTrajStart+j+1),xSubTraj(:,j+1),u(:,knotsSubTrajStart+j+1));
    J=J+g*dt;
    dgdx=dg(:,1+(1:nXc));
    dgdu=dg(:,1+nXc+(1:nU));
    dJdalphaSubTraj{nSubTraj}=dJdalphaSubTraj{nSubTraj}+dgdx*PsubTraj{nSubTraj}*dt+dgdu*du2dalphaSubTraj*dt+g*ddtdalphaSubTraj;
end
[h,dh]=finalCostFun(t(nT),xSubTraj(:,end));
J=J+h;
dhdt=dh(:,1);
dhdx=dh(:,1+(1:nXc));
dJdalphaSubTraj{nSubTraj}=dJdalphaSubTraj{nSubTraj}+dhdx*PsubTraj{nSubTraj};
dJdalpha(1)=dJdalphaSubTraj{nSubTraj}(1)+dJdalpha(1)+dhdt;
dJdalpha(1+(nSubTraj-1)*nXc+(1:nXc))=dJdalphaSubTraj{nSubTraj}(1+(1:nXc))+dJdalpha(1+(nSubTraj-1)*nXc+(1:nXc));
dJdalpha(1+nXc*nSubTraj+nU*knotsSubTrajStart+(1:nU*(nKnotsInSubTraj(nSubTraj))))=dJdalpha(1+nXc*nSubTraj+nU*knotsSubTrajStart+(1:nU*(nKnotsInSubTraj(nSubTraj))))...
    +dJdalphaSubTraj{nSubTraj}(1+nXc+(1:nU*(nKnotsInSubTraj(nSubTraj))));
G(1:(1+nXc*nSubTraj+nT*nU))=dJdalpha';
f(1)=J;
if(isfield(con,'xf'))
    if(isfield(con.xf,'lb')||isfield(con.xf,'ub'))
        f=[f;xSubTraj(:,nKnotsInSubTraj(nSubTraj))];
        G=[G;reshape(PsubTraj{nSubTraj},[],1)];
    end
    if(isfield(con.xf,'ceq'))
        [c,dc]=geval(con.xf.ceq,xSubTraj(:,nKnotsInSubTraj(nSubTraj)));
        f=[f;c];
        G=[G;reshape(dc*PsubTraj{nSubTraj},[],1)];
    end
end
end

function [xNew,dxndt,dxndx,dxndu1,dxndu2,dxnddt]=rungeKuttaDynamics(sys,t,x,u1,u2,dt)
%Assumes the control tape is first order hold
nU=sys.getNumInputs();
nXc=sys.getNumContStates();
[f1,df1]=geval(@sys.dynamics,t,x,u1);
df1dt=df1(:,1);
df1dx=df1(:,1+(1:nXc));
df1du=df1(:,1+nXc+(1:nU));
k1=f1*dt;
dk1dt=df1dt*dt;
dk1dx=df1dx*dt;
dk1du1=df1du*dt;
dk1du2=zeros(size(df1du));
dk1ddt=f1;
[f2,df2]=geval(@sys.dynamics,t+0.5*dt,x+0.5*k1,(u1+u2)/2);
df2dt=df2(:,1);
df2dx=df2(:,1+(1:nXc));
df2du=df2(:,1+nXc+(1:nU));
k2=f2*dt;
dk2dt=(df2dt+df2dx*(0.5*dk1dt))*dt;
dk2dx=df2dx*(eye(nXc)+0.5*dk1dx)*dt;
dk2du1=(df2dx*(0.5*dk1du1)+0.5*df2du)*dt;
dk2du2=(df2dx*(0.5*dk1du2)+0.5*df2du)*dt;
dk2ddt=f2+(df2dt*0.5+df2dx*0.5*dk1ddt)*dt;
[f3,df3]=geval(@sys.dynamics,t+0.5*dt,x+0.5*k2,(u1+u2)/2);
df3dt=df3(:,1);
df3dx=df3(:,1+(1:nXc));
df3du=df3(:,1+nXc+(1:nU));
k3=f3*dt;
dk3dt=(df3dt+df3dx*0.5*dk2dt)*dt;
dk3dx=(df3dx*(eye(nXc)+0.5*dk2dx))*dt;
dk3du1=(df3dx*0.5*dk2du1+df3du*0.5)*dt;
dk3du2=(df3dx*0.5*dk2du2+df3du*0.5)*dt;
dk3ddt=f3+(df3dt*0.5+df3dx*0.5*dk2ddt)*dt;
[f4,df4]=geval(@sys.dynamics,t+dt,x+k3,u2);
df4dt=df4(:,1);
df4dx=df4(:,1+(1:nXc));
df4du=df4(:,1+nXc+(1:nU));
k4=f4*dt;
dk4dt=(df4dt+df4dx*dk3dt)*dt;
dk4dx=(df4dx*(eye(nXc)+dk3dx))*dt;
dk4du1=(df4dx*dk3du1)*dt;
dk4du2=(df4dx*dk3du2+df4du)*dt;
dk4ddt=f4+(df4dt+df4dx*dk3ddt)*dt;
xNew=x+1/6*(k1+2*k2+2*k3+k4);
dxndt=1/6*(dk1dt+2*dk2dt+2*dk3dt+dk4dt);
dxndx=eye(nXc)+1/6*(dk1dx+2*dk2dx+2*dk3dx+dk4dx);
dxndu1=1/6*(dk1du1+2*dk2du1+2*dk3du1+dk4du1);
dxndu2=1/6*(dk1du2+2*dk2du2+2*dk3du2+dk4du2);
dxnddt=1/6*(dk1ddt+2*dk2ddt+2*dk3ddt+dk4ddt);
end

function [utraj,xtraj]=rtrl_wrapup(sys,w,nT,nXc,nU,nSubTraj,nKnotsInSubTraj,t0)
tspan=w(1);
dt=tspan/(nT-1);
xSubTrajStart=reshape(w(1+(1:nXc*nSubTraj)),nXc,nSubTraj);
u=reshape(w(1+nXc*nSubTraj+(1:nU*nT)),nU,nT);
x=zeros(nXc,nT);
xdot=zeros(nXc,nT);
t=t0:dt:(t0+dt*(nT-1));
for i=1:nSubTraj
    knotSubTrajStart=sum(nKnotsInSubTraj(1:i))-nKnotsInSubTraj(i);
    x(:,knotSubTrajStart+1)=xSubTrajStart(:,i);
    for j=1:nKnotsInSubTraj(i)-1
        x(:,knotSubTrajStart+j+1)=rungeKuttaDynamics(sys,t(knotSubTrajStart+j),x(:,knotSubTrajStart+j),u(:,knotSubTrajStart+j),u(:,knotSubTrajStart+j+1),dt);
        xdot(:,knotSubTrajStart+j)=sys.dynamics(t(knotSubTrajStart+j),x(:,knotSubTrajStart+j),u(:,knotSubTrajStart+j));
    end
end
xdot(:,nT)=sys.dynamics(t(nT),x(:,nT),u(:,nT));
xtraj=PPTrajectory(pchipDeriv(t,x,xdot));
xtraj=setOutputFrame(xtraj,sys.getStateFrame);
utraj=PPTrajectory(foh(t,u));
utraj=setOutputFrame(utraj,sys.getInputFrame);
end
