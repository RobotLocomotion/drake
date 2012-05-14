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
      persistent uidStore objectStore;
      
      if (isempty(objectStore))
        uidStore={};
        objectStore={};
        ind=0;
      else
        [~,ind]=ismember(uid,uidStore);
      end
      
      if ind<1
        uidStore{end+1}=uid;
        objectStore{end+1}=obj;
      else
        if (length(ind)>1) error('found multiple matches in the object store.  this shouldn''t happen'); end
        obj = objectStore{ind};
      end
    end
  end
end
