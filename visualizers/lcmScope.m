function lcmScope(robotname)

if (nargin>0)
  typecheck(robotname,'char');
else
%  robotname = '.*';  % need to fix varname_index for this to work
  error('you must specify a robot name: lcmScope(robotname)');
end
checkDependency('lcm_enabled');

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
%aggregator.setMaxMessages(1);  % make it a last-message-only queue

varname_index = length(robotname)+length('_scope_')+1;
lc.subscribe([lower(robotname),'_scope_.*'],aggregator);

scope_data=[];
max_scope_id = 1;
scope_rows = 1;
scope_cols = 1;

warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
h=figure(103); clf;
set(h,'Name',['LCM Scope: ',robotname],'NumberTitle','off','MenuBar','none','Renderer','OpenGL','Toolbar','none','DoubleBuffer','on'); 
setAlwaysOnTop(h,true); 
hold on; 

while true
  msg = getNextMessage(aggregator,0);
  if (isempty(msg)) 
    drawnow;
    while(isempty(msg))  % after I've drawn once, block for long periods until a new message arrives
      msg = getNextMessage(aggregator,1000);  
    end
  end
  
  channel = char(msg.channel);
  varname = channel(varname_index:end);
  data = robotlib.shared.lcmt_scope_data(msg.data);

  breset = false;
  if (isempty(scope_data) || ~isfield(scope_data,varname))
    breset = true;
  else
    d = getfield(scope_data,varname);
    if (fieldchanged(d,data))
      breset = true;
      delete(d.handle);
    end
  end

  if (breset)  % reset the current variable.  redraw the current axis.
    t = data.timestamp/1000;
    val = data.data;
    num_points = data.num_points;
    d=[];
    d.t = repmat(data.timestamp/1000,1,num_points);
    d.val = repmat(data.data(:),1,num_points);
    d.scope_id = data.scope_id;

    breplot_all = false;
    if (d.scope_id>max_scope_id)
      max_scope_id = d.scope_id;
      scope_rows = ceil(sqrt(max_scope_id));
      scope_cols = ceil(max_scope_id/scope_rows);
      clf;
      breplot_all = true;
    end
    d.linespec = char(data.linespec);
    d.handle = [];
    scope_data = setfield(scope_data,varname,d);
    
    if (breplot_all)
      f = fieldnames(scope_data);
      for i=1:length(f)
        scope_data = plotTrace(scope_data,f{i},scope_rows,scope_cols);
      end
    else
      scope_data = plotTrace(scope_data,varname,scope_rows,scope_cols);
    end
  else
    d.t = [d.t(2:end),data.timestamp/1000];
    d.val = [d.val(:,2:end),data.data(:)];
    scope_data = setfield(scope_data,varname,d);
    set(d.handle,'XData',d.t,'YData',d.val);
  end
  
end  
  
  
end

function scope_data = plotTrace(scope_data,varname,scope_rows,scope_cols)
  d = getfield(scope_data,varname);
  hsubfig = subplot(scope_rows,scope_cols,d.scope_id);
  hold on;
  d.handle=plot(d.t,d.val,d.linespec);
  [legh,objh,outh,outm] = legend(hsubfig);
  if (isempty(outh)) legend(d.handle,varname);
  else
    keep = ishandle(outh);
    legend([outh(keep);d.handle],{outm{keep},varname});
  end
  scope_data = setfield(scope_data,varname,d);
end

function bchanged = fieldchanged(scope_var,data)
  bchanged = (data.timestamp/1000 < scope_var.t(end)) || ...
    (data.scope_id ~= scope_var.scope_id) || ...
    (data.datalen ~= size(scope_var.val,1)) || ...
    (data.num_points ~= length(scope_var.t)) || ...
    ~strcmpi(char(data.linespec),scope_var.linespec);
end

