function [c,model] = parseMaterial(model,node,options)

name=char(node.getAttribute('name'));
name=regexprep(name, '\.', '_', 'preservecase');

c = .7*[1 1 1];

% look up material
if length(model.material)>0
  ind = strmatch(lower(name),lower({model.material.name}),'exact');
  if (~isempty(ind))
    c=model.material(ind).c;
  end
end

colornode = node.getElementsByTagName('color').item(0);
if ~isempty(colornode) && colornode.hasAttribute('rgba')
  c = str2num(char(colornode.getAttribute('rgba')));
  c = c(1:3);
end

if ~isempty(name)
  model.material = [model.material,struct('name',name,'c',c)];
end
end

