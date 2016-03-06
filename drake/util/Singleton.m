classdef Singleton < handle
  % Implements a singleton design pattern.  Classes derived from this class
  % can only be instantiated one at at time.
  
  methods 
    function obj=Singleton(uid)
      if (nargin<1)
        uid = class(obj);  % by default, use the class name as the uid
      end
      
      obj = Singleton.getSingleton(obj,uid);
    end
  end
  
  methods (Static=true,Access=private)
    function obj = getSingleton(obj,uid)
      persistent uidMap;
      if isempty(uidMap)
        uidMap = containers.Map('UniformValues',false);
      end

      if ~isKey(uidMap, uid)
        uidMap(uid) = obj;
      else
        if strcmp(class(obj), class(uidMap(uid)))
          obj = uidMap(uid);
        else
          warning('found object in the store with the same uid, but a different type.  overwriting the store with the new object');
          uidMap(uid) = obj;
        end
      end
    end
  end
end
