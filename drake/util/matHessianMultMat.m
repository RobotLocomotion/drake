function ddAB = matHessianMultMat(A,B,dA,dB,ddA,ddB)
% Given matrix A,B, and their gradient and Hessian, compute the Hessain of
% A*B
% A,dA,ddA; B,dB,ddB; and ddAB are in the geval format
[A_rows,A_cols] = size(A);
[B_rows,B_cols] = size(B);
if(A_cols ~= B_rows)
  error('The number of columns in A should be equal to the number of rows in B');
end
dA_size = size(dA);
if(dA_size(1) ~= A_rows*A_cols)
  error('dA has incorrect size');
end
nX = dA_size(2);
dB_size = size(dB);
if(dB_size(1) ~= B_rows*B_cols || dB_size(2) ~= nX)
  error('dB has incorrect size');
end
ddA_size = size(ddA);
if(ddA_size(1) ~= A_rows*A_cols || ddA_size(2) ~= nX*nX)
  error('ddA has incorrect size');
end
ddB_size = size(ddB);
if(ddB_size(1) ~= B_rows*B_cols || ddB_size(2) ~= nX*nX)
  error('ddB has incorrect size');
end
ddAB = zeros(A_rows*B_cols,nX^2);
for i = 1:nX
  for j = 1:nX
    ddAB_col_idx = (i-1)*nX+j;
    ddAB(:,ddAB_col_idx) = ddAB(:,ddAB_col_idx) + reshape(reshape(ddA(:,(i-1)*nX+j),A_rows,A_cols)*B,[],1);
    ddAB(:,ddAB_col_idx) = ddAB(:,ddAB_col_idx) + reshape(reshape(dA(:,i),A_rows,A_cols)*reshape(dB(:,j),B_rows,B_cols),[],1)+reshape(reshape(dA(:,j),A_rows,A_cols)*reshape(dB(:,i),B_rows,B_cols),[],1);
    ddAB(:,ddAB_col_idx) = ddAB(:,ddAB_col_idx) + reshape(A*reshape(ddB(:,(i-1)*nX+j),B_rows,B_cols),[],1);
  end
end
end