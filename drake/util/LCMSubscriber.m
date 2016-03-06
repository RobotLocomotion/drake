classdef LCMSubscriber < handle
% defines an interface for LCM objects that get wrapped up in matlab
% vectors

% note: i don't like that this has to derive from handle, but 
% "If a class defines super-classes, all or none must be handle classes"
% and CoordinateFrames implement this.  if it's a problem later, I might be
% able to have a second class LCMSubscriberHandle < LCMSubscriber & handle
  
  methods (Abstract)
    obj = subscribe(obj,channel)
    [x,t] = getNextMessage(obj,timeout)   % x=t=[] if timeout
    [x,t] = getCurrentValue(obj)
    str = defaultChannel(obj);
  end
end

