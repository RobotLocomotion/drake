function polysys = makeTrigPolySystem(sys,options)

% tries to convert the current system into a trig-poly system
%
% @option replace_output_w_new_state makes the new system have an output method 
% which is just the full state in the new coordinate system.  this
% addresses the problem that a trig system that was using the state as the
% output probably didn't output sin(theta) and cos(theta) as the outputs
% (but it could have!).  @default: false
%
% @option rational_dynamics_numerator For models that can only be written as a
% rational trig-poly system, pass a function handle here that supplies the f(t,x,u) 
% part of e(x)*xdot = f(t,x,u).  the original dynamics function will only be
% called if this field is empty.  @default: []
%
% @option rational_dynamics_denominator For models that can only be written as a
% rational trig-poly system, pass a function handle here that supplies the e(x) part of 
% e(x)*xdot = f(t,x,u).  the original dynamics function will only be
% called if this field is empty.  @default: []
%
% @option unit_circle_constraint_tau Time constant of the unit circle
% constraint.  Defines the dynamics off the unit circle to converge to the 
% unit circle, which helps with numerical stability.  @default: 1

if (nargin<2) options=struct(); end
if (~isfield(options,'replace_output_w_new_state')) options.replace_output_w_new_state = false; end
if (~isfield(options,'rational_dynamics_numerator')) options.rational_dynamics_numerator = []; end
if (~isfield(options,'rational_dynamics_denominator')) options.rational_dynamics_denominator = []; end
if (~isfield(options,'unit_circle_constraint_tau')) options.constraint_tau = 1; end 

if (isempty(options.rational_dynamics_numerator) + isempty(options.rational_dynamics_denominator) == 1)
  error('must specify both numerator and denominator if specifying rational dynamics'); 
end

t=msspoly('t',1);
q=msspoly('q',sys.num_x);
s=msspoly('s',sys.num_x);

c=msspoly('c',sys.num_x);
x=TrigPoly(q,s,c);
u=msspoly('u',sys.num_u);

tp_dynamics_rhs=[];
tp_dynamics_lhs=[];
if (sys.num_xc>0)
  if (~isempty(options.rational_dynamics_numerator))
    tp_dynamics_rhs = options.rational_dynamics_numerator(t,x,u);
    tp_dynamics_lhs = options.rational_dynamics_denominator(x);
    sizecheck(tp_dynamics_rhs,[sys.num_xc,1]);
    sizecheck(tp_dynamics_lhs,[sys.num_xc,sys.num_xc]);
  else
    tp_dynamics_rhs = sys.dynamics(t,x,u);
  end
end

if (sys.num_xd>0)
  tp_update = sys.update(t,x,u);
else
  tp_update=[];
end

if (~options.replace_output_w_new_state && sys.num_y>0)
  tp_output = sys.output(t,x,u);
else
  tp_output=[];
end

if (sys.num_xcon>0) 
  error('not implemented yet, but should be easy'); 
end

all_methods=[tp_dynamics_lhs(:);tp_dynamics_rhs;tp_update;tp_output];
if (~isTrigOrPoly(all_methods))
  tp_dynamics_lhs
  tp_dynamics_rhs
  tp_update
  tp_output
  error('dynamics, update, and/or output methods depend both trigonometrically and polynomially on a single variable');
end
all_methods=getmsspoly(all_methods);

%% determine new x vector from the elements of x,s, and c, that are actually used

 
% I have e(x)xdot=f(x), and want E(y)ydot=F(y), where y=g(x).
% We'll exploit that dim(y)>=dim(x)
% I get the implicit dynamics from
% ydot=G(x)*inv(e(x))*f(x), where G(x)=dg/dx
%  => e(x)*inv[G(x)^T*G(x)] G(x)^T ydot = f(x)
% ...  for my g(x), inv[G(x)^T*G(x)]=I.
% the additional equations come from the unit circle constraints, which add
%  phi(y)=0 := s^2+c^2=1 => s*sdot+c*cdot=0 := Phi*y=0 with Phi(i,:) = [0...0 s c 0 ... 0]
% but for stability, I impose phidot(y) = -alpha*phi(y)

xnew=[];
G=msspoly(zeros(sys.num_x)); Phi=msspoly([]); phidot=msspoly([]);
unit_circle_constraints=[];
for i=1:sys.num_x
  if (deg(all_methods,s(i))>0 || deg(all_methods,c(i))>0)
    xnew=[xnew;s(i);c(i)];
    if (i<=sys.num_xd)
      error('trig discrete variables not supported yet.  (have to think about if it''s even reasonable...).'); 
    else
      yind=size(xnew,1)-1;
      G(yind,i)=c(i);
      G(yind+1,i)=-s(i);
      cind=size(unit_circle_constraints,1)+1;
      Phi(cind,i)=2*s(i);
      Phi(cind,i+1)=2*c(i);
      phidot(cind,1) = -(1/options.constraint_tau)*(s(i)^2 + c(i)^2 - 1);
    end
    unit_circle_constraints=[unit_circle_constraints;s(i)^2+c(i)^2-1];
  else % leave it as poly 
    xnew=[xnew;q(i)];
    if (i>sys.num_xd)
      G(end+1,i)=1;
    end
  end
end
if (length(xnew)~=sys.num_x)  % only do this if there are some trig vars
  if (isempty(tp_dynamics_lhs))
    tp_dynamics_lhs = 0*x(1)+eye(sys.num_xc);
  end
  if (size(Phi,2)<length(xnew)) Phi(end,length(xnew))=0; end  % zero-pad if necessary
  tp_dynamics_lhs = [tp_dynamics_lhs*G'; Phi];  % e(x)*G(x)';
  tp_dynamics_rhs=[tp_dynamics_rhs;phidot];
end
p_xnew=msspoly('x',length(xnew));

if (sys.num_xc)
  p_dynamics_rhs = subs(getmsspoly(tp_dynamics_rhs),xnew,p_xnew);
else
  p_dynamics_rhs=[];
end
if (sys.num_xd)
  p_update = subs(getmsspoly(tp_update),xnew,p_xnew);
else
  p_update=[];
end
if (options.replace_output_w_new_state)
  p_output = p_xnew;
elseif sys.num_y
  p_output = subs(getmsspoly(tp_output),xnew,p_xnew);
else
  p_output=[];
end
unit_circle_constraints=subs(unit_circle_constraints,xnew,p_xnew);

polysys = PolynomialSystem(size(G,1),length(p_update),sys.num_u,length(p_output),sys.isDirectFeedthrough(),sys.isTI(),p_dynamics,p_update,p_output);
if (sys.num_u)
  polysys = setInputLimits(polysys,sys.umin,sys.umax);
end
polysys = polysys.addStateConstraints(unit_circle_constraints);
if (~isempty(tp_mass_matrix))
  polysys = polysys.setMassMatrix(subs(getmsspoly(tp_mass_matrix),xnew,p_xnew));
end

