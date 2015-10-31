function lcmScope(robotname,exitMatlabOnClose)

if (nargin<1)
  robotname = '.*';
else
  typecheck(robotname,'char');
end
if (nargin<2)
  exitMatlabOnClose = false;
end

checkDependency('lcm');

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(10);  % make it a last-message-only queue

lc.subscribe([lower(robotname),'_scope_.*'],aggregator);

scope_data=[];
max_scope_id = 1;
scope_rows = 1;
scope_cols = 1;

warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');
h=figure(103); clf; set(h,'Color',[1 1 1]);
set(h,'Name',['LCM Scope: ',robotname],'NumberTitle','off','Renderer','OpenGL','DoubleBuffer','on','MenuBar','none','Toolbar','none'); 
if (exitMatlabOnClose) set(h,'CloseRequestFcn','delete(gcf); exit;'); end
setAlwaysOnTop(h,true); 
hold on; 

while true
  msg = getNextMessage(aggregator,0);
  if (isempty(msg)) 
    drawnow;
    while(isempty(msg))  % after I've drawn once, block for long periods until a new message arrives
      msg = getNextMessage(aggregator,100);
      drawnow;
    end
  end
  
  channel = char(msg.channel);
  varname = channel((strfind(channel,'_scope_')+7):end);
  data = drake.lcmt_scope_data(msg.data);

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
    num_points = data.num_points;
    d=[];
    d.x = repmat(data.xdata,1,num_points);
    d.y = repmat(data.ydata,1,num_points);
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
    d.x = [d.x(2:end),data.xdata];
    d.y = [d.y(2:end),data.ydata];
    scope_data = setfield(scope_data,varname,d);
    set(d.handle,'XData',d.x,'YData',d.y);
  end
  
end  
  
  
end

function scope_data = plotTrace(scope_data,varname,scope_rows,scope_cols)
  d = getfield(scope_data,varname);
  hsubfig = subplot(scope_rows,scope_cols,d.scope_id);
  hold on;
  d.handle=plot(d.x,d.y,d.linespec);
%   [legh,objh,outh,outm] = legend(hsubfig);
%   if (isempty(outh)) legend(d.handle,varname);
%   else
%     keep = ishandle(outh);
%     legend([outh(keep);d.handle],{outm{keep},varname});
%   end
  scope_data = setfield(scope_data,varname,d);
end

function bchanged = fieldchanged(scope_var,data)
  bchanged = (data.resetOnXval && data.xdata < scope_var.x(end)) || ...
    (data.scope_id ~= scope_var.scope_id) || ...
    (data.num_points ~= length(scope_var.x)) || ...
    ~strcmpi(char(data.linespec),scope_var.linespec);
end

