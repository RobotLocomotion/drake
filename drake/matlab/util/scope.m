function scope(robotname,varname,xval,yval,options)
% todo: add support for different figures/subfigures
global g_scope_enable;

if (isempty(g_scope_enable)) g_scope_enable=true; end

if (g_scope_enable && checkDependency('lcm'))
  
  if (nargin<5) options = struct();
  else typecheck(options,'struct'); end
  if (~isfield(options,'scope_id')) options.scope_id = 1;
  else typecheck(options.scope_id,'double'); end
  if (~isfield(options,'linespec')) options.linespec = '';
  else typecheck(options.linespec,'char'); end
  if (~isfield(options,'num_points')) options.num_points = 200; end
  if (~isfield(options,'resetOnXval')) options.resetOnXval = true; end
  
  typecheck(robotname,'char');
  typecheck(varname,'char');
  typecheck(xval,'double');
  typecheck(yval,'double');
  if (length(xval)>1 ) error('x values should be scalars'); end
  
  for i = 1:length(yval)
      varnameExt = [varname num2str(i)];
      lc = lcm.lcm.LCM.getSingleton();
      msg = drake.lcmt_scope_data();
      msg.scope_id = options.scope_id;
      msg.xdata = xval;
      msg.ydata = yval(i);
      msg.linespec = options.linespec;
      msg.num_points = options.num_points;
      msg.resetOnXval = options.resetOnXval;
      lc.publish([lower(robotname),'_scope_',varnameExt],msg);
  end
end

end
