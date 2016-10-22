function taylorVarScalarTest
for count=1:1000,
  m = randi(20);
  n = randi(20);
    
  A = randn(m,n);
  b = randn(1);
  
  % Test A*b
  [y, dy] = geval(@mult,A,b);
  
  if ~isequal(reshape(dy(:,end),m,n),A)
    error('Derivative d/db (A*b) should be A')
  end
  
  for i=1:m,
    for j=1:n,
      %Geval returns in column major order
      k = (j-1)*m + i;
      tmp = zeros(m,n);
      tmp(i,j) = b;
      if ~isequal(tmp,reshape(dy(:,k),m,n))
        error('Derivative d/dA(i,j) (A*b) should be b')
      end
    end
  end
  
  %Test b*A
  [y, dy] = geval(@mult,b,A);
  if ~isequal(reshape(dy(:,1),m,n),A)
    error('Derivative d/db (A*b) should be A')
  end
  
  for i=1:m,
    for j=1:n,
      %Geval returns in column major order
      k = (j-1)*m + i;
      tmp = zeros(m,n);
      tmp(i,j) = b;
      if ~isequal(tmp,reshape(dy(:,k+1),m,n))
        error('Derivative d/dA(i,j) (A*b) should be b')
      end
    end
  end
end

  function y = mult(a,b)
    y = a*b;
  end
end