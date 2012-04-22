function h=plotFunnel(V,x0,plot_dims,options)
% Plots the one-level set of V
%
% @param V funnel to plot. Can either be a msspoly or a PolynomialTrajectory
% @param x0 either a point or a trajectory to plot along
% @param plot_dims dimensions of V to plot given as a 1x2 array
%   @default [1, 2]
% @param options options structure
%
% @option inclusion = { 'slice' | 'projection' }
%    'slice' -- include the points in the plane of the given
%        dimensions for which V(x) < 1.
%    'projection' -- plot the projection of {x | V(x) < 1} into the
%        given plane.
%
% @retval h column vector of handles for any graphics objects created

% todo: support wrapping coordinates

if (nargin<3 || isempty(plot_dims)) plot_dims = [1,2]; end
if (nargin<4) options=struct(); end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end
if (~isfield(options,'inclusion'))
        options.inclusion = 'slice';
end

hold on;
view(0,90);


if (isa(V,'msspoly'))
  sizecheck(V,[1 1]);
  typecheck(x0,'double');  % not actually a trajectory
   
  p_x=decomp(V);
  if (length(x0)~=length(p_x)) error('need to handle this case better'); end
  no_plot_dims=1:length(x0);  no_plot_dims(plot_dims)=[];
  
  % TODO: Here we split between projection and slice.
  if strcmp(options.inclusion,'slice')
      if (~isempty(no_plot_dims))
          V=subs(V,p_x(no_plot_dims),x0(no_plot_dims));
      end
      
      x=getLevelSet(V,x0(plot_dims),struct('tol',options.tol));
  elseif strcmp(options.inclusion,'projection')
      x=getProjection(V,x0,plot_dims,struct('tol',options.tol));
  else
      error(['Unknown inclusion method: ' options.inclusion]);
  end

  h=fill3(x(1,:),x(2,:),repmat(0,1,size(x,2)),options.color,'LineStyle','-','LineWidth',2);

else
  typecheck(V,'PolynomialTrajectory');
  sizecheck(V,[1 1]);

  ts = V.getBreaks();
  hold on;

  h=[];
  for i=length(ts)-1:-1:1
    xfun0 = getLevel(ts(i),V,plot_dims,options);
    xfun1 = getLevel(ts(i+1)-eps,V,plot_dims,options);
    
    if (i==length(ts)-1)
      % draw level-set at the end
      plot3(xfun1(1,:),xfun1(2,:),repmat(.1,1,size(xfun1,2)),'k','LineWidth',2);
    end
    
    x = [xfun0,xfun1(:,end:-1:1)];
    k = convhull(x(1,:),x(2,:));
    h=[h;fill3(x(1,k),x(2,k),repmat(0,1,length(k)),options.color,'LineStyle','none')];
    h=[h;plot3(x(1,k),x(2,k),repmat(-.1,1,length(k)),'k','LineWidth',5)];
    
    h=[h;plot3(xfun0(1,:),xfun0(2,:),repmat(.1,1,size(xfun0,2)),'k','LineWidth',.5)];
  end
  
  % draw level-set at the beginning
  h=[h;plot3(xfun0(1,:),xfun0(2,:),repmat(.1,1,size(xfun0,2)),'k','LineWidth',2)];
end

end

  function x=getLevel(t,Vtraj,plot_dims,options)
    p_x = Vtraj.p_x;
    no_plot_dims = 1:length(p_x);  no_plot_dims(plot_dims)=[];
    V = Vtraj.eval(t);
    if (deg(V,p_x)>2) error('only works for quadratics'); end
    H = doubleSafe(0.5*diff(diff(V,p_x)',p_x));
    b = -0.5*(H\doubleSafe(subs(diff(V,p_x),p_x,0*p_x)'));
    
    if strcmp(options.inclusion,'slice')
        if (~isempty(no_plot_dims))
            V = subs(V,p_x(no_plot_dims),b(no_plot_dims));
            b=b(plot_dims);
        end
        x=getLevelSet(V,b,struct('tol',options.tol));
    elseif strcmp(options.inclusion,'projection')
        x=getProjection(V,b,plot_dims,struct('tol',options.tol));
    else
        error(['Unknown inclusion method: ' options.inclusion]);
    end
  end


function y=doubleSafe(x)
  y=double(x);
  if (~isa(y,'double')) error('double failed'); end
end
