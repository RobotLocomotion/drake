function [y] = matGradMult(A,b,transposeA)
% [y] = matGradMult(A,b)
% If A is the result of dB/dX for matrix B and vector X, as output by
% geval, then return A*b (a matrix).
% @param vector b
% @param gradient of A wrt. a vector, written as a matrix
% geval)
% @return A*b

if exist('transposeA','var')
  if transposeA
    n = length(b);
    m = size(A,2);
    k = size(A,1)/n;
    y = zeros(m,n);
    for i=1:m,
      y(:,i) = reshape(A(:,i),n,k)'*b;
    end
    return;
  end
end
n = length(b);
m = size(A,2);
k = size(A,1)/n;

B = sparse(repmat((1:k)',n,1),1:k*n,reshape(repmat(b',k,1),k*n,1),k,k*n);
y = B*A;
% B_row=repmat(reshape(diag([b;zeros(k-n-1,1)]),1,(k-1)^2),1,k);
% B =  reshape(B_row(1:k*k*n),k*n,k)';
% y = B*A;
% return

% tensor = reshape(A,k,n,m);  tensor method was actually a little slower
% % (<10% difference)
% y = zeros(k,m); %was mxn-->changed to nxm then changed again
% for i=1:m,  %was n
%   y(:,i) = reshape(A(:,i),k,n)*b;
% %   y(:,i) = tensor(:,:,i)*b;
% end
end
