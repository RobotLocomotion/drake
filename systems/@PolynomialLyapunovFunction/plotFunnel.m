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

typecheck(x0,'double');  % not actually a trajectory

if (length(x0)~=V.frame.dim) error('need to handle this case better'); end
no_plot_dims=1:length(x0);  no_plot_dims(plot_dims)=[];
  
% TODO: Here we split between projection and slice.
if strcmp(options.inclusion,'slice')
  if (~isempty(no_plot_dims))
    subV=PolynomaialLyapunovFunction(V.frame,subs(V,V.frame.poly(no_plot_dims),x0(no_plot_dims)));
  else
    subV=V;
  end
  
  x=getLevelSet(subV,x0(plot_dims),struct('tol',options.tol));
elseif strcmp(options.inclusion,'projection')
  x=getProjection(V,x0,plot_dims,struct('tol',options.tol));
else
  error(['Unknown inclusion method: ' options.inclusion]);
end

h=fill3(x(1,:),x(2,:),repmat(0,1,size(x,2)),options.color,'LineStyle','-','LineWidth',2);

