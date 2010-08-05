function lcmScope(robotname)

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
aggregator.setMaxMessages(1);  % make it a last-message-only queue

varname_index = length(robotname)+length('_scope_')+1;
lc.subscribe([lower(robotname),'_scope_thetaDot'],aggregator);

scope_data=[];

num_points = 200;  % todo: get this from lcm message

h=figure(1); set(h,'WindowStyle','modal'); hold on; 

while true
  if (aggregator.numMessagesAvailable()>0)
    msg = getNextMessage(aggregator);
    
    channel = char(msg.channel);
    varname = channel(varname_index:end);
    data = robotlib.shared.lcmt_scope_data(msg.data);
    t = data.timestamp/1000;
    val = data.data;

    if (isempty(scope_data) || ~isfield(scope_data,varname))
      d = struct('t',repmat(t,1,num_points),'val',repmat(val(:),1,num_points),'handle',[]);
      d.handle=plot(d.t,d.val);
      legend(d.handle,varname);
      scope_data = setfield(scope_data,varname,d);
    else
      d = getfield(scope_data,varname);
      if (t<d.t(end)) % if t is less than the tape, clear the tape and start over
        d.t = repmat(t,1,num_points);
        d.val = repmat(val(:),1,num_points);
      else
        d.t = [d.t(2:end),t];
        d.val = [d.val(:,2:end),val(:)];
      end
      scope_data = setfield(scope_data,varname,d);
    end
    
    set(d.handle,'XData',d.t,'YData',d.val);
      
    drawnow;
  end
end


