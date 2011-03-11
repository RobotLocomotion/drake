function [w0,wlow,whigh,Flow,Fhigh,A,iAfun,jAvar,iGfun,jGvar,userfun,wrapupfun,iname,oname]=...
    rtrlSNOPTtranscription(sys,costFun,finalCostFun,x0,utraj0,con,options)
%Can only handle the equality final value constraint I*xf=con.xf.beq; so
%con.xf.Aeq has to be identity matrix.
%The options are left vacant. To-do: to implement the gradient test.
nU=sys.getNumInputs();
nX=sys.getNumContStates();
t=utraj0.getBreaks();
u=utraj0.eval(t);
nT = length(t);

w0=vec(u);
A=[]; iAfun=[]; jAvar=[];
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
whigh=repmat(con.u.ub,nT,1);
wlow=repmat(con.u.lb,nT,1);
if(~isfield(con.xf,'ub')&&~isfield(con.xf,'lb'))
    iGfun=ones((nT)*nU,1);
    jGvar=(1:(nT)*nU)';
    Fhigh=inf;
    Flow=-inf;
else
    iGfun=vec(((1:(nX+1))'*ones(1,(nT)*nU))');
    jGvar=repmat((1:(nT)*nU)',1+nX,1);
    Fhigh=[inf;con.xf.ub];
    Flow=[-inf;con.xf.lb];
end
iname={};
oname={};

userfun=@(w) rtrl_userfun(sys,w,costFun,finalCostFun,t,nX,nU,x0,con,options);
wrapupfun=@(w) rtrl_wrapup(sys,w,t,nX,nU,x0);
end


function [f,G]=rtrl_userfun(sys,w,costFun,finalCostFun,t,nX,nU,x0,con,options)
nT=length(t);
dJdalpha=zeros(1,(nT)*nU);
u=reshape(w,nU,nT);
x=zeros(nX,nT);
x(:,1)=x0;
finalStateConstraintEnable=(isfield(con.xf,'lb')|isfield(con.xf,'ub'));
if(finalStateConstraintEnable)
     f=zeros(1+nX,1);
     G=zeros((1+nX)*nU*(nT),1);
else
     f=zeros(1);
     G=zeros(nU*(nT),1);
end
J=0;
P=zeros(nX,(nT)*nU);
dt=t(2)-t(1);
for i=1:nT-1
    [f,df]=geval(@sys.dynamics,t(i),x(:,i),u(:,i));
    dfdx=df(:,1+(1:nX));
    dfdu=df(:,1+nX+(1:nU));
    F_x=dfdx;
    dudalpha=zeros(nU,(nT)*nU);
    dudalpha(:,(i-1)*nU+(1:nU))=eye(nU);
    F_alpha=dfdu*dudalpha;
    [g,dgc]=geval(costFun,t(i),x(:,i),u(:,i));
    dgdx=dgc(:,1+(1:nX));
    dgdu=dgc(:,1+nX+(1:nU));
    G_x=dgdx;
    G_alpha=dgdu*dudalpha;
    x(:,i+1)=x(:,i)+f.*dt;
    dJdalpha=dJdalpha+(G_x*P+G_alpha).*dt;
    P=P+(F_x*P+F_alpha).*dt;
    J=J+g.*dt;
end
[h,dh]=geval(finalCostFun,t(nT),x(:,nT));
J=J+h;
dhdx=dh(:,1+(1:nX));
dJdalpha=dJdalpha+dhdx*P;
G(1:(nT)*nU)=dJdalpha';
if(finalStateConstraintEnable)
    G((nT)*nU+(1:nX*(nT)*nU))=vec(P');
    f(1+(1:nX))=x(:,nT);
end
f(1)=J;
end


function [utraj,xtraj]=rtrl_wrapup(sys,w,t,nX,nU,x0)
nT=length(t);
x=zeros(nX,nT);
x(:,1)=x0;
u=reshape(w,nU,nT);
dt=t(2)-t(1);
for i=1:nT-1
    x(:,i+1)=x(:,i)+sys.dynamics(t(i),x(:,i),u(:,i)).*dt;
end
xtraj=PPTrajectory(zoh(t,x));
utraj=PPTrajectory(zoh(t,u));
end