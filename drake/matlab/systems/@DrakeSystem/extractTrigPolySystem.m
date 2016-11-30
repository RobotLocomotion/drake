function polysys = extractTrigPolySystem(sys,options)

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

if (nargin<2), options=struct(); end
if (~isfield(options,'replace_output_w_new_state')), options.replace_output_w_new_state = false; end
if (~isfield(options,'rational_dynamics_numerator')), options.rational_dynamics_numerator = []; end
if (~isfield(options,'rational_dynamics_denominator')), options.rational_dynamics_denominator = []; end
if (~isfield(options,'unit_circle_constraint_tau')), options.constraint_tau = 1; end 

if (isempty(options.rational_dynamics_numerator) + isempty(options.rational_dynamics_denominator) == 1)
  error('must specify both numerator and denominator if specifying rational dynamics'); 
end

checkDependency('spotless');

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

if ~isempty(sys.state_constraints)
  warning('state constraints not implemented yet, but some should be easy'); 
end

all_methods=[tp_dynamics_lhs(:);tp_dynamics_rhs;tp_update;tp_output];
if (~isTrigOrPoly(all_methods))
  tp_dynamics_lhs  %#ok
  tp_dynamics_rhs  %#ok
  tp_update        %#ok
  tp_output        %#ok
  error('dynamics, update, and/or output methods depend both trigonometrically and polynomially on a single variable');
  % did you perhaps need to use the "replace_output_w_new_state" option?
end
all_methods=getmsspoly(all_methods);

%% determine new x vector from the elements of x,s, and c, that are actually used

 
% I have e(x)xdot=f(x), and want E(y)ydot=F(y), where y=g(x).
% We'll exploit that dim(y)>=dim(x)
% I get the implicit dynamics from
%   ydot = G(x)*xdot, where G(x) = dg/dx
%     (note that G(x) here is orthonormal, i.e. G(x)'*G(x) = I)
%   G(x)'*ydot = xdot
%   =>  e(x)*G(x)'*ydot = f(x)
% the trigpoly class gives me these in terms of y
%   =>  e(y)*G(y)'*ydot = f(y)
% the additional equations come from the unit circle constraints, which add
%  phi(y)=0 := s^2+c^2-1 => 2*s*sdot+2*c*cdot=0 := Phi(y)*ydot=0 with Phi(i,:) = [0...0 2*s 2*c 0 ... 0]
% but for stability, I impose Phi(y)*ydot = -alpha*phi(y)

xnew=[];  %#ok<*AGROW>
G=msspoly(zeros(sys.num_x)); Phi=msspoly([]); phidot=msspoly([]);
unit_circle_constraints=[];
sin_ind=false(sys.num_x); cos_ind=sin_ind; x_ind=sin_ind;
name=sys.getStateFrame.getCoordinateNames(); newname={};
for i=1:sys.num_x
  if (deg(all_methods,s(i))>0 || deg(all_methods,c(i))>0)
    xnew=[xnew;s(i);c(i)];  
    yind=size(xnew,1)-1;
    sin_ind(yind,i)=true;
    cos_ind(yind+1,i)=true;
    newname{yind}=['sin(',name{i},')'];  
    newname{yind+1}=['cos(',name{i},')']; 
    if (i<=sys.num_xd)
      error('trig discrete variables not supported yet.  (have to think about if it''s even reasonable...).'); 
    else
      G(yind,i)=c(i);
      G(yind+1,i)=-s(i);
      cind=size(unit_circle_constraints,1)+1;
      Phi(cind,yind)=2*s(i);
      Phi(cind,yind+1)=2*c(i);
      phidot(cind,1) = -(1/options.constraint_tau)*(s(i)^2 + c(i)^2 - 1);
    end
    unit_circle_constraints=[unit_circle_constraints;s(i)^2+c(i)^2-1]; 
  else % leave it as poly 
    xnew=[xnew;q(i)];  
    x_ind(length(xnew),i)=true;
    newname{length(xnew)}=name{i};
    if (i>sys.num_xd)
      G(length(xnew),i)=1;  
    end
  end
end
if (length(xnew)~=sys.num_x)  % only do this if there are some trig vars
  if isempty(tp_dynamics_lhs)
    tp_dynamics_lhs = 0*x(1)+eye(sys.num_xc);
  end
  
  if (size(Phi,2)<length(xnew)), Phi(end,length(xnew))=0; end  % zero-pad if necessary
  if (size(sin_ind,1)<length(xnew)), sin_ind(length(xnew),end)=false; end  % zero-pad
  if (size(cos_ind,1)<length(xnew)), cos_ind(length(xnew),end)=false; end
  if (size(x_ind,1)<length(xnew)), x_ind(length(xnew),end)=false; end
  
  tp_dynamics_lhs = [tp_dynamics_lhs*G'; Phi];  % e(x)*G(x)';
  tp_dynamics_rhs=[tp_dynamics_rhs;phidot];

% an experiment: don't add phi to dynamics. 
%  tp_dynamics_lhs = tp_dynamics_lhs*G';
%  tp_dynamics_rhs= tp_dynamics_rhs;
end

% set up new coordinate frame
tpframe = CoordinateFrame(['TP',sys.getStateFrame().name],length(xnew),'x');
tpframe.setCoordinateNames(newname);
p_xnew=tpframe.getPoly;

% set up transformations between the frames
sys.getStateFrame.addTransform(TrigToPolyTransform(sys.getStateFrame,tpframe,sin_ind,cos_ind,x_ind));
tpframe.addTransform(PolyToTrigTransform(tpframe,sys.getStateFrame,sin_ind,cos_ind,x_ind));

% now populate the dynamics methods using the new frame variables
if (sys.num_xc)
  p_dynamics_rhs = subs(getmsspoly(tp_dynamics_rhs),xnew,p_xnew);
  if isempty(tp_dynamics_lhs)
    p_dynamics_lhs=[];
  else
    p_dynamics_lhs = subs(getmsspoly(tp_dynamics_lhs),xnew,p_xnew);
  end
else
  p_dynamics_rhs=[];
  p_dynamics_lhs=[];
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

if (options.replace_output_w_new_state && sys.num_y>0)
  outframe = tpframe;
else
  outframe = sys.getOutputFrame;
end

polysys = SpotPolynomialSystem(sys.getInputFrame,tpframe,outframe,p_dynamics_rhs,p_dynamics_lhs,p_update,p_output,unit_circle_constraints);
if (sys.num_u)
  polysys = setInputLimits(polysys,sys.umin,sys.umax);
end


