function y = homogTransMult(T,x)

% applies the homogeonous transform T to x

y = T*[x;1+0*x(1,:)];
%y = y(1:end-1,:)./repmat(y(end,:),size(y,1)-1,1);
y = bsxfun(@rdivide,y(1:end-1,:),y(end,:)); % this is faster than repmat
