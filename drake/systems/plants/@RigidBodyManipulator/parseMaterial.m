function [c,options] = parseMaterial(node,options)
% @ingroup URDF Parsing

name=char(node.getAttribute('name'));
name=regexprep(name, '\.', '_', 'preservecase');

c = .7*[1 1 1];

if ~isfield(options,'material') options.material=[]; end

% look up material
if length(options.material)>0
  ind = strmatch(lower(name),lower({options.material.name}),'exact');
  if (~isempty(ind))
    c=options.material(ind).c;
  end
end

colornode = node.getElementsByTagName('color').item(0);
if ~isempty(colornode) && colornode.hasAttribute('rgba')
  c = str2num(char(colornode.getAttribute('rgba')));
  c = c(1:3);
end

if ~isempty(name)
  options.material = [options.material,struct('name',name,'c',c)];
end

% NOTEST
end

