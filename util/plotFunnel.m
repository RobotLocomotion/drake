function plotFunnel(xtraj,V,plot_dims,options)

% plots the one-level set of V

% todo: support wrapping coordinates

if (nargin<3 || isempty(plot_dims)) plot_dims = [1,2]; end
if (nargin<4) options=struct(); end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end

typecheck(V,'PolynomialTrajectory');
sizecheck(V,[1 1]); 

ts = V.getBreaks();
p_x = V.p_x;
no_plot_dims = 1:length(p_x);  no_plot_dims(plot_dims)=[]; 

hold on; 
view(0,90);

for i=length(ts)-1:-1:1
  t0 = ts(i); 
  t1 = ts(i+1)-eps;
  
  x0 = xtraj.eval(t0); 
  x1 = xtraj.eval(t1);
  
  V0 = V.eval(t0); 
  V1 = V.eval(t1);

  % find the true zeros
  if (deg(V0,p_x)>2 || deg(V1,p_x)>2) error('only works for quadratics'); end
  H0 = double(0.5*diff(diff(V0,p_x)',p_x));
  b0 = -0.5*(H0\double(subs(diff(V0,p_x),p_x,0*p_x)'));
  H1 = double(0.5*diff(diff(V1,p_x)',p_x));
  b1 = -0.5*(H1\double(subs(diff(V1,p_x),p_x,0*p_x)'));
  
  if (~isempty(no_plot_dims))
    V0 = subs(V0,p_x(no_plot_dims),b0(no_plot_dims));
    V1 = subs(V1,p_x(no_plot_dims),b0(no_plot_dims));
    x0=x0(plot_dims);
    x1=x1(plot_dims);
    b0=b0(plot_dims);
    b1=b1(plot_dims);
  end
 
  xfun0=getLevelSet(V0,b0,struct('tol',options.tol)); 
  xfun0=xfun0+repmat(x0,1,size(xfun0,2));  
  xfun1=getLevelSet(V1,b1,struct('tol',options.tol));
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
end

% draw level-set at the beginning
plot3(xfun0(1,:),xfun0(2,:),repmat(.1,1,size(xfun0,2)),'k','LineWidth',2);
