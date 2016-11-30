function cell_str = cellStrCat(varargin)
  % cell_str = cellStrCat(cell_str_1, cell_str_2, ...) - Creates a new
  % cell array of strings from a list of cell arrays of strings by by
  % recursively concatenating their contents
  %
  % @param cell_str_i - Cell array of strings
  % @retval cell_str - Cell array of combined strings
  %
  % Example
  %
  % >> cellStrCat({'a','b'},{'1','2'},'_',{'x','y'})
  %
  % ans =
  %
  % 'a1_x'   'a1_y'   'a2_x'   'a2_y'   'b1_x'   'b1_y'   'b2_x'   'b2_y'
  %
  cell_str1 = varargin{1};
  if ischar(cell_str1), cell_str1 = {cell_str1}; end;
  if length(varargin) == 2
    cell_str2 = varargin{2};
    cell_str = cellfun(@(str) strcat(str,cell_str2),cell_str1, ...
                       'UniformOutput',false);
    cell_str = [cell_str{:}];
  else
    cell_str = cellStrCat(cell_str1,cellStrCat(varargin{2:end}));
  end
end

