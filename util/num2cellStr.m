function cell_str = num2cellStr(num)
  % cell_str = num2cellStr(num) - Converts a numerical array (of any
  % dimensions) into a cell array of strings of the same size in which
  % 
  %     cell_str{i,j,...} = num2str(num(i,j,...))
  %
  % @param num - Numerical array of any size
  % @retval cell_str - Cell array of strings in which each element is
  % the num2str equivalent of the corresponding elment of num.
  %
  % Example:
  %
  % >> num2cellStr(1:4)
  %
  % ans =
  %
  % '1'    '2'    '3'    '4'
  %
  cell_str = cellfun(@num2str,num2cell(num),'UniformOutput',false);
end
