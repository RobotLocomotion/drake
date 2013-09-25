function dAinv = invMatGrad(A,dA)
% A is an invertible matrix, dA = dAdx, where x is a vector, dA = [dAdx1
% dAdx2,...,dAdxn]
% Ainv = inv(A)
% dAinv = [dAinvdx1 dAinvdx2 ... dAinvdxn]
if(abs(cond(A))>1e15)
    error('matrix A has condition number %f, it is not invertible',cond(A));
end
[rowA,colA] = size(A);
if(rowA ~= colA)
    error('The A matrix must be square');
end
nX = size(dA,2);
dA = reshape(dA,rowA,colA*nX);
invAdA = A\dA;
invAdAtranspose = reshape(permute(reshape(invAdA,colA,colA,nX),[2,1,3]),colA,colA*nX)';
% dAinv = -reshape(permute(reshape((A')\invAdAtranspose,colA,colA,nX),[2,1,3]),colA*colA,nX);
dAinv = -reshape(permute(reshape((invAdAtranspose/A)',colA,colA,nX),[2,1,3]),colA*colA,nX);
end