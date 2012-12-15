classdef LCMPublisher < handle
% defines an interface for an object that can publish a double on an LCM
% channel.

% note: i don't like that this has to derive from handle, but 
% "If a class defines super-classes, all or none must be handle classes"
% and CoordinateFrames implement this.  if it's a problem later, I might be
% able to have a second class LCMPublisherHandle < LCMPublisher & handle

  methods (Abstract)
    publish(obj,t,x,channel);
    str = defaultChannel(obj);
  end
end
