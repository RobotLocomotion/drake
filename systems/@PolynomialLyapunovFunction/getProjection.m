function y=getProjection(V,x0,plot_dims,options)
    typecheck(V,'msspoly');
    x = decomp(V);    
    if (nargin<4) options=struct(); end
    if (~isfield(options,'tol')) options.tol = 2e-3; end % default tolerance of fplot
    if (~isfield(options,'method')) options.method = 'hull'; end
    
    if (deg(V,x)>2) 
        error('Only projections of quadratics supported.');
    end
    % put in the form x'Qx + 2c'x + b
    c = double(subs(diff(V,x),x,0*x)/2)';
    Q = double(diff(diff(V,x)',x))/2;
    b = double(subs(V,x,0*x));
    
    d = - Q\c;
    v = double(subs(V,x,d));
    if v > 1, y = []; return; end
    Q = Q/(1-v);
    % First, construct a set of directions:
    N = 100;
    X = zeros(length(x0),N);
    if length(plot_dims) == 2
        thetas = linspace(-pi,pi,N);
        X(plot_dims,:) = [cos(thetas); sin(thetas)];
    else
        X(plot_dims,:) = randn(length(plot_dims),N); 
    end
    % For each direction, max. f'x subj. to x'Sx <= 1 is equivalent to
    %                     max. f'x subj. to x'Sx == 1 ''
    %                     max. f'S^(-1/2)x  subj. to y'y == 1
    extrm = (Q\X)./repmat(sqrt(sum(X.*(Q\X))),length(x0),1);
    y = repmat(d(plot_dims),1,N) + extrm(plot_dims,:);
end