function nullptr = nullPointer()
% Return an appropriate value which can be passed in to mex as a null pointer

[~, maxArraySize] = computer();
is64bit = maxArraySize > 2^31;
if is64bit
  nullptr = uint64(0);
else
  nullptr = uint32(0);
end


