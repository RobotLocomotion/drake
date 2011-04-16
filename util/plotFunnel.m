function plotFunnel(V,x0,plot_dims,options)

% plots the one-level set of V

% todo: support wrapping coordinates

if (nargin<3 || isempty(plot_dims)) plot_dims = [1,2]; end
if (nargin<4) options=struct(); end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end


if (isa(V,'msspoly'))
  sizecheck(V,[1 1]);
  typecheck(x0,'double');  % not actually a trajectory
 
  p_x=decomp(V);
  if (length(x0)~=length(p_x)) error('need to handle this case better'); end
  no_plot_dims=1:length(x0);  no_plot_dims(plot_dims)=[];
  
  if (~isempty(no_plot_dims))
    V=subs(V,p_x(no_plot_dims),x0(no_plot_dims));
  end
  x=getLevelSet(V,x0(plot_dims),struct('tol',options.tol));

  fill3(x(1,:),x(2,:),repmat(0,1,size(x,2)),options.color,'LineStyle','-','LineWidth',2);

else
  typecheck(V,'PolynomialTrajectory');
  sizecheck(V,[1 1]);

  ts = V.getBreaks();

  hold on;
  view(0,90);
  
  for i=length(ts)-1:-1:1
    xfun0 = getLevel(ts(i),V,plot_dims,options);
    xfun1 = getLevel(ts(i+1)-eps,V,plot_dims,options);
    
    if (i==length(ts)-1)
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
end

end

  function x=getLevel(t,Vtraj,plot_dims,options)
    p_x = Vtraj.p_x;
    no_plot_dims = 1:length(p_x);  no_plot_dims(plot_dims)=[];
    V = Vtraj.eval(t);
    if (deg(V,p_x)>2) error('only works for quadratics'); end
    H = doubleSafe(0.5*diff(diff(V,p_x)',p_x));
    b = -0.5*(H\doubleSafe(subs(diff(V,p_x),p_x,0*p_x)'));

    if (~isempty(no_plot_dims))
      V = subs(V,p_x(no_plot_dims),b(no_plot_dims));
      b=b(plot_dims);
    end
    x=getLevelSet(V,b,struct('tol',options.tol));
  end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end
