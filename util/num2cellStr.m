function cell_str = num2cellStr(num)
  cell_str = cellfun(@num2str,num2cell(num),'UniformOutput',false);
end
