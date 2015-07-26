function output = cell2matmsspoly(C)
output = C{1};
for i = 2:1:length(C)
   output = [output C{i}]; 
end

end