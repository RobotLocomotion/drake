function str = ccodeWithEigenRef(path, filename, function_name_cell, output_cell, inputs_cell)
  if ~iscell(function_name_cell)
    function_name_cell = {function_name_cell};
  end
  if ~iscell(output_cell)
    output_cell = {output_cell};
  end
  if ~iscell(inputs_cell)
    inputs_cell = {inputs_cell};
  end
  assert(numel(function_name_cell) == numel(output_cell));
  assert(numel(function_name_cell) == numel(inputs_cell));
  function_string_cell = cell(size(function_name_cell));
  header_string_cell = cell(size(function_name_cell));
  for i = 1:numel(function_name_cell)
    [function_string_cell{i}, header_string_cell{i}] = generateFunctionString(function_name_cell{i}, output_cell{i}, inputs_cell{i});
  end

  % Write .h file
  include = sprintf('#include <Eigen/Dense>');
  body = sprintf('%s\n', header_string_cell{:});
  str = sprintf('%s\n\n%s', include, body);
  fid = fopen(fullfile(path, [filename '.h']), 'w');
  fprintf(fid, str);

  % Write .cpp file
  include = sprintf('#include "%s.h"\n', filename);
  body = sprintf('%s\n\n', function_string_cell{:});
  str = sprintf('%s\n\nusing namespace Eigen;\n\n%s', include, body);
  fid = fopen(fullfile(path, [filename '.cpp']), 'w');
  fprintf(fid, str);
end

function [function_string, header_string] = generateFunctionString(function_name, output, inputs)
  assert(isfield(output,'name'))
  assert(ischar(output.name))
  assert(isfield(output, 'var'))
  assert(isa(output.var, 'sym'))
  input_vars = sym([]);
  for i = 1:numel(inputs)
    assert(isfield(inputs(i),'name'))
    assert(ischar(inputs(i).name))
    assert(isfield(inputs(i), 'var'))
    assert(isa(inputs(i).var, 'sym'))
    input_vars = [input_vars; inputs(i).var(:)]; %#ok
  end
  assert(all(ismember(symvar(output.var), input_vars)));

  tmp_file = tempname;
  ccode(output.var, 'file', tmp_file);
  body = fileread(tmp_file);
  body = replaceOutputName(body, output);
  input_args_cell = {};
  for i = 1:numel(inputs)
    for j = 1:numel(inputs(i).var)
      expression = char(inputs(i).var(j));
      if numel(inputs(i).var) == 1
        replace = inputs(i).name;
      else
        replace = [inputs(i).name, sprintf('(%d)', j-1)];
      end
      body = regexprep(body, expression, replace);
    end
    input_args_cell{end+1} = [argumentTypeFromSym(inputs(i).var, true), ' ', inputs(i).name]; %#ok
  end
  input_args = strjoin(input_args_cell, ', ');
  output_args = argumentTypeFromSym(output.var, false);
  signature = sprintf('%s %s(%s)', output_args, function_name, input_args);
  intermediate_vars = unique(regexp(body, 't\d+', 'match'));
  variables = [sprintf('  double %s;\n', intermediate_vars{:}), sprintf('  %s %s;\n', output_args, output.name)];
  header_string = sprintf('%s;', signature);
  signature = regexprep(signature, 'Eigen::', '');
  function_string = sprintf('%s\n{\n%s\n%s\n  return %s;\n}', signature, variables, body, output.name);
end

function arg_type = argumentTypeFromSym(var, is_input)
  if numel(var) == 1
    arg_type = 'double';
  else
    var_size = size(var);
    base_type = sprintf('Eigen::Matrix<double, %d, %d>', var_size);

    if var_size(1) <= 4
      if var_size(2) == 1
        base_type = sprintf('Eigen::Vector%dd', var_size(1));
      elseif var_size(1) == var_size(2)
        base_type = sprintf('Eigen::Matrix%dd', var_size(1));
      end
    end

    if is_input
      arg_type = sprintf('const Eigen::Ref<const %s>&', base_type);
    else
      arg_type = base_type;
    end
  end
end

function body = replaceOutputName(body, output)
  if numel(output.var) == 1
    expression = 't0';
    replace = output.name;
  else
    expression = '\s*A0\[(\d*)\]\[(\d*)\]';
    replace = sprintf('%s($1,$2)', output.name);
  end
  body = regexprep(body, expression, replace);
  body = regexprep(body, sprintf(';(%s)', output.name), sprintf(';\n  $1'));
end
