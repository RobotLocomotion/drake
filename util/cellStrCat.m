function cell_str = cellStrCat(varargin)
  % Output a cell array of string
  cell_str1 = varargin{1};
  if ischar(cell_str1), cell_str1 = {cell_str1}; end;
  if length(varargin) == 2
    cell_str2 = varargin{2};
    cell_str = cellfun(@(str) strcat(str,cell_str2),cell_str1,'UniformOutput',false);
    cell_str = [cell_str{:}];
  else
    cell_str = cellStrCat(cell_str1,cellStrCat(varargin{2:end}));
  end
end

