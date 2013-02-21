function ext = objext()

if ispc()
  ext = 'obj';
else
  ext = 'o';
end