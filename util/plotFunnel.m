function plotFunnel(xtraj,V,options)

% plots the one-level set of V

% todo: support more than 2d
% todo: support wrapping coordinates

if (nargin<3) options=struct(); end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end

typecheck(V,'Trajectory');
v0 = V.eval(V.tspan(1));
typecheck(v0,'msspoly');
sizecheck(v0,[1 1]); 

ts = V.getBreaks();
p_t = msspoly('t',1);

hold on; 
view(0,90);

for i=length(ts)-1:-1:1
  t0 = ts(i); 
  t1 = ts(i+1)-eps;
  
  x0 = xtraj.eval(t0);
  x1 = xtraj.eval(t1);
  
  V0 = subs(V.eval(t0),p_t,t0);
  V1 = subs(V.eval(t1),p_t,t1);

  xfun0=getLevelSet(V0,[],struct('tol',options.tol)); 
  xfun0=xfun0+repmat(x0,1,size(xfun0,2));  
  xfun1=getLevelSet(V1,[],struct('tol',options.tol));
  xfun1=xfun1+repmat(x1,1,size(xfun1,2));
  
  if (i==length(ts))
    % draw level-set at the end
    plot3(xfun1(1,:),xfun1(2,:),repmat(.1,1,size(xfun1,2)),'k','LineWidth',2);
  end

  x = [xfun0,xfun1(:,end:-1:1)];
  k = convhull(x(1,:),x(2,:));
  fill3(x(1,k),x(2,k),repmat(0,1,length(k)),options.color,'LineStyle','none');
  plot3(x(1,k),x(2,k),repmat(-.1,1,length(k)),'k','LineWidth',5);

  plot3(xfun0(1,:),xfun0(2,:),repmat(.1,1,size(xfun0,2)),'k','LineWidth',.5);
  drawnow;
end

% draw level-set at the beginning
plot3(xfun0(1,:),xfun0(2,:),repmat(.1,1,size(xfun0,2)),'k','LineWidth',2);
