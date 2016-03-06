function z=atan2(y,x)

if isa(y,'sym')||isa(x,'sym')
  z = atan(y,x);
else
  z = builtin('atan2',y,x);
end