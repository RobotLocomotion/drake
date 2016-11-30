function mat = arraylist2mat(list)
% converts a java.util.ArrayList<Double) into a matlab double vector

mat = zeros(list.size(),1);
for i=1:length(mat)
  mat(i)=list.get(i);
end

