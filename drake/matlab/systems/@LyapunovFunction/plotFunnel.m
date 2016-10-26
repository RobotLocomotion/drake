function h=plotFunnel(obj,options)
% Plots the one-level set of V
%
% @param options options structure
%
% @option plotdims coordinates along which to plot given as a 1x2 array
%   @default [1, 2]
% @option x0 default coordinates (e.g. to be used in the slice)
% @option inclusion = { 'slice' | 'projection' }
%    'slice' -- include the points in the plane of the given
%        dimensions for which V(x) < 1.
%    'projection' -- plot the projection of {x | V(x) < 1} into the
%        given plane.
% @option color matlab color for the funnel @default [.7 .7 .7]
% @option tol tolerance for computing the level set
%
% @retval h column vector of handles for any graphics objects created

% todo: support wrapping coordinates

if (nargin<2) options=struct(); end
if ~isfield(options,'plotdims') options.plotdims = [1;2]; end
if ~isfield(options,'x0') options.x0 = zeros(obj.getFrame.dim,1); end  
if (~isfield(options,'inclusion')) options.inclusion = 'slice'; end
if (~isfield(options,'color')) options.color=.7*[1 1 1]; end
if (~isfield(options,'tol')) options.tol = .01; end

hold on;
view(0,90);

if isTI(obj) 
  % TODO: Here we split between projection and slice.
  if strcmp(options.inclusion,'slice')
    x=getLevelSet(obj,0,options);
  elseif strcmp(options.inclusion,'projection')
    x=getProjection(obj,0,options.x0,options.plotdims,options);
  else
    error(['Unknown inclusion method: ' options.inclusion]);
  end

  h=fill3(x(1,:),x(2,:),repmat(0,1,size(x,2)),options.color,'LineStyle','-','LineWidth',2);
  coords = obj.getFrame.getCoordinateNames();
  xlabel(coords{options.plotdims(1)});
  ylabel(coords{options.plotdims(2)});
else
  if ~isfield(options,'ts') error('you must specify the time samples for this system using options.ts'); end
  ts = options.ts;
  
  if isnumeric(options.x0) options.x0 = ConstantTrajectory(options.x0); end
  x0 = options.x0;
  
  h=[];
  for i=length(ts)-1:-1:1
    if strcmp(options.inclusion,'slice')
      options.x0 = x0.eval(ts(i));
      xfun0 = getLevelSet(obj,ts(i),options);
      options.x0 = x0.eval(ts(i+1)-eps);
      xfun1 = getLevelSet(obj,ts(i+1)-eps,options);
    elseif strcmp(options.inclusion,'projection')
      xfun0 = getProjection(obj,ts(i),x0.eval(ts(i)),options.plotdims,options);
      xfun1 = getProjection(obj,ts(i+1)-eps,x0.eval(ts(i+1)-eps),options.plotdims,options);
    else
      error(['Unknown inclusion method: ' options.inclusion]);
    end
    
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

