function list=mat2arraylist(mat)
% converts a matlab vector to a java.util.ArrayList<Double> 

mat=mat(:);
list = java.util.ArrayList(length(mat));
for i=1:length(mat)
  list.add(mat(i));
end
