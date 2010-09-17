function snopt_gradtest(x,USERFUN,iGfun,jGvar,options)

if (nargin<5) options = struct(); end
if (~isfield(options,'N')) options.N = 21; end
if (~isfield(options,'scale')) options.scale = ones(size(x)); end
if (~isfield(options,'threshold')) options.threshold = .01; end
if (~isfield(options,'iname')) options.iname = {}; end

[f,G] = feval(USERFUN,x);

if ((length(iGfun) ~= length(jGvar)) || (length(iGfun) ~= length(G)))
  size(iGfun)
  size(jGvar)
  size(G)
  error('Something is wrong.  G, iGfun, and jGvar should all be the same size');
end
if (max(iGfun)>length(f)) error('iGfun references non-existent fun'); end
if (max(jGvar)>length(x)) error('jGvar references non-existent var'); end

for n=1:length(iGfun)
  i=iGfun(n); j=jGvar(n);
  samples = linspace(-options.scale(j),options.scale(j),options.N);
  for s=1:options.N
    y(:,s) = feval(USERFUN,x+[zeros(j-1,1);samples(s);zeros(length(x)-j,1)]);
  end

  hN = ceil(options.N/2);
  if (abs(y(i,hN-1)-f(i)-G(n)*samples(hN-1))>options.threshold)  % some threshold on plotting
    sfigure(109);
    plot(x(j)+samples,y(i,:),x(j),f(i),'r*');
    if (isempty(options.iname))
      xlabel(['a(',num2str(j),')']);
    else
      xlabel(options.iname{j});
    end
    ylabel(['f(',num2str(i),')']);
    line(x(j)+options.scale(j)*[-1,1],f(i)+options.scale(j)*G(n)*[-1,1],'color',[1 0 0]);
    axis tight;
%    axis equal;
    drawnow;
    pause;
  end
end
