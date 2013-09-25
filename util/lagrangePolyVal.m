function p = lagrangePolyVal(t,x,y)
% Use Barycentric Formula to compute the value of the lagrange polynomial,
% which is a polynomial interpolation at points x with values y. The code is
% vectorized, please refer to the paper 'Barycentric Lagrange
% Interpolation' by Jean-Paul Berrut and Lloyd N Trefethen for more
% information
% t     -- a row vector, compute the value of the polynomial at t(i)
% x     -- a row vector, the mesh points
% y     -- a matrix, each row of y, y(i,:), represents i'th polynomial.
%          y(i,j) is the value of the i'th polynomial at the mesh point x(j) 
% p     -- a matrix, p(i,j) is the value of the i'th lagrange polynomial at
%          t(j)
% For example, if we have two polynomials p1 = x^3+x+1 and p2 = x^2-x, the
% mesh points are x = [1 3 5 7], the mesh values for p1 is y(1,:) = [3 31 131 351], the
% mesh values for p2 is y(2,:) = [0 6 20 42], we want to compute the value of p1 and
% p2 at the points t = [1 2 3 4], then p = lagrangePolyVal(t,x,y) will
% return as p(1,:) = [3 11 31 69] and p(2,:) = [0 2 6 12], exactly the
% values of p1 and p2 computed at t.
t = t(:)';
x = x(:)';
nX = length(x);
nY = size(y,1);
if(size(y,2)~= nX)
    error('The size of the mesh points x and the mesh values y do not match')
end
nt = length(t);
% check if some t resides at the mesh points. If so, compute seperately
tx = repmat(t',1,nX);
xt = repmat(x,nt,1);
[iEq,jEq] = find(tx==xt);
p = zeros(nY,nt);
p(:,iEq) = y(:,jEq);
if(length(iEq) == nt)
    % If all t are mesh points, return the corresponding mesh values;
    return;
else
    ind = 1:nt;
    iNeq = ind(logical(prod(tx-xt,2)'));
    tn = t(iNeq);
    ntn = length(tn);
    xx = repmat(x',1,nX);
    % omega is the barycentric weights
    omega = ones(1,nX)./prod(xx-xx'+eye(nX),2)';
    % den is the denominator of the barycentric formula
    den = repmat(omega',1,ntn)./(repmat(tn,nX,1)-repmat(x',1,ntn));
    % To multiply with y, repmat the denominator [den(1,:);den(2,:);...] to
    % [den(1,:);den(1,:);...den(1,:);den(2,:);den(2,:);...]
    if(nY>1)
        den = reshape(permute(reshape(repmat(reshape(permute(reshape(den',ntn,1,nX),[2,1,3]),1,[]),nY,1),nY,ntn,nX),[2,1,3]),ntn,nX*nY)';
    end
    nom = den.*repmat(y(:),1,ntn);
    p(:,iNeq) = reshape((sum(permute(reshape(nom,nY,nX,ntn),[2 3 1]),1)./sum(permute(reshape(den,nY,nX,ntn),[2 3 1]),1)),ntn,nY)';
end
end
