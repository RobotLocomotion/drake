function [value, syms] = symbolify(s, prefix)
  % takes an arbitary (non-class) object, and replaces all leaf variables
  % with symbols, all starting with the name $prefix.
  %
  % The second return value is a mapping of matlab syntax to symbols
  %
  % For example
  %   [v, syms] = symbolify({[1 1], struct('a', 1)})
  % Gives
  %   v{1}(1) == syms('{1}(1)')
  %   v{1}(2) == syms('{1}(2)')
  %   v{2}.a  == syms('{2}.a')
  syms = containers.Map;
  if iscell(s)
    value_items = {};
    for i = 1:prod(size(s))
      [value_items{i}, newsyms] = symbolify(s{i}, [prefix, '_', num2str(i)]);
      for k = keys(newsyms)
        k = k{1};
        syms(sprintf('{%i}%s', i, k)) = newsyms(k);
      end
    end
    value = reshape(value_items, size(s));
  elseif ~isscalar(s)
    value_items = [];
    for i = 1:prod(size(s))
      [newvalue, newsyms] = symbolify(s(i), [prefix, '_', num2str(i)]);
      value_items = [value_items, newvalue];
      for k = keys(newsyms)
        k = k{1};
        syms(sprintf('(%i)%s', i, k)) = newsyms(k);
      end
    end
    value = reshape(value_items, size(s));
  elseif isnumeric(s)
    value = sym(prefix, 'real');
    syms('') = value;
  elseif isstruct(s)
    value = struct();

    for field=fieldnames(s)'
      field = field{1};
      [value.(field), newsyms] = symbolify(s.(field), [prefix, '_', field]);
      for k = keys(newsyms)
        k = k{1};
        syms(sprintf('.%s%s', field, k)) = newsyms(k);
      end
    end
  else
    error('Could not symbolize value')
  end
end
