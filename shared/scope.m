function scope(robotname,varname,scope_id,t,val)

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