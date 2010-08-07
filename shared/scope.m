function scope(robotname,varname,scope_id,t,val)

typecheck(robotname,'char');
typecheck(varname,'char');
typecheck(scope_id,'double'); 
if (length(t)>1) error('t should be a scalar'); end
typecheck(val,'double');  

% todo: add support for different figures/subfigures
global g_scope_enable;

if (isempty(g_scope_enable)) g_scope_enable=true; end

if (g_scope_enable)
  lc = lcm.lcm.LCM.getSingleton();
  
  msg = robotlib.shared.lcmt_scope_data();
  msg.timestamp = 1000*t;
  msg.scope_id = 1;
  msg.data = val(1);

  lc.publish([lower(robotname),'_scope_',varname],msg);
end

end