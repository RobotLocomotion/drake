classdef PassByValueMap
  % This class provides a pass-by-value version of the containers.Map
  % class. See the documentation for that class for more description and
  % instructions. PassByValueMap should not be used for large,
  % frequently changing maps, as any modification to the map creates a
  % copy.
  
  properties (Access = private)
    map
  end
  methods
    function obj = PassByValueMap(varargin)
      obj.map = containers.Map(varargin{:});
    end

    function disp(obj)
      disp(obj.map)
    end

    % Pass throughs
    function [varargout] = isKey(obj, varargin)
      varargout = cell(1, nargout);
      [varargout{:}] = obj.map.isKey(varargin{:});
    end

    function [varargout] = keys(obj, varargin)
      varargout = cell(1, max(1,nargout));
      [varargout{:}] = obj.map.keys(varargin{:});
    end

    function [varargout] = values(obj, varargin)
      varargout = cell(1, nargout);
      [varargout{:}] = obj.map.values(varargin{:});
    end

    function [varargout] = size(obj, varargin)
      varargout = cell(1, nargout);
      [varargout{:}] = obj.map.size(varargin{:});
    end

    function [varargout] = length(obj, varargin)
      varargout = cell(1, nargout);
      [varargout{:}] = obj.map.length(varargin{:});
    end

    function [varargout] = isempty(obj, varargin)
      varargout = cell(1, nargout);
      [varargout{:}] = obj.map.isempty(varargin{:});
    end

    function count = Count(obj)
      count = obj.map.Count();
    end

    function key_type = KeyType(obj)
      key_type = obj.map.KeyType();
    end

    function value_type = ValueType(obj)
      value_type = obj.map.ValueType();
    end

    % Modified methods
    function varargout = subsref(obj, S)
      varargout = cell(1, max(1, nargout));
      if S(1).type == '.'
        [varargout{:}] = builtin('subsref', obj, S);
      else
        [varargout{:}] = subsref(obj.map, S);
      end
    end

    function obj = subsasgn(obj, varargin)
      obj.map = obj.getCopyOfInternalMap();
      obj.map = obj.map.subsasgn(varargin{:});
    end

    function obj = remove(obj, varargin)
      obj.map = obj.getCopyOfInternalMap();
      obj.map = obj.map.remove(varargin{:});
    end

    function new_map = getCopyOfInternalMap(obj)
      new_map = containers.Map('KeyType', obj.map.KeyType, ...
        'ValueType', obj.map.ValueType);
      if ~isempty(obj.map.keys)
        keys = obj.map.keys;
        for i = 1:numel(obj.map.keys)
          new_map(keys{i}) = obj.map(keys{i});
        end
      end
    end
  end
end
