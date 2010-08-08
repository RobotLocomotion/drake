function scope(robotname,varname,scope_id,t,val,linespec,num_points)

% todo: add support for different figures/subfigures
global g_scope_enable;

if (isempty(g_scope_enable)) g_scope_enable=true; end

if (g_scope_enable || ~checkDependency('lcm_enabled'))
  typecheck(robotname,'char');
  typecheck(varname,'char');
  typecheck(scope_id,'double'); 
  if (length(t)>1) error('t should be a scalar'); end
  typecheck(val,'double');  

  lc = lcm.lcm.LCM.getSingleton();
  
  msg = robotlib.shared.lcmt_scope_data();
  msg.timestamp = 1000*t;
  msg.scope_id = scope_id;
  
  msg.datalen = prod(size(val));
  msg.data = val(:);
  
  if (nargin>5) 
    typecheck(linespec,'char');
    msg.linespec = linespec;
  else
    msg.linespec = '';
  end

  if (nargin>6)
    msg.num_points = num_points;
  else
    msg.num_points = 200;  % just some reasonable default
  end
  
  lc.publish([lower(robotname),'_scope_',varname],msg);
end

end