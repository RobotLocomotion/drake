function [A,b,Aeq,beq]=vert2lcon(V,tol)
%An extension of Michael Kleder's vert2con function, used for finding the 
%linear constraints defining a polyhedron in R^n given its vertices. This 
%wrapper extends the capabilities of vert2con to also handle cases where the 
%polyhedron is not solid in R^n, i.e., where the polyhedron is defined by 
%both equality and inequality constraints.
% 
%SYNTAX:
%
%  [A,b,Aeq,beq]=vert2lcon(V,TOL)
%
%The rows of the N x n matrix V are a series of N vertices of a polyhedron
%in R^n. TOL is a rank-estimation tolerance (Default = 1e-10).
%
%Any point x inside the polyhedron will/must satisfy
%  
%   A*x  <= b
%   Aeq*x = beq
%
%up to machine precision issues.
%
%
%EXAMPLE: 
%
%Consider V=eye(3) corresponding to the 3D region defined 
%by x+y+z=1, x>=0, y>=0, z>=0.
%
% 
%   >>[A,b,Aeq,beq]=vert2lcon(eye(3))
%
%
%     A =
% 
%         0.4082   -0.8165    0.4082
%         0.4082    0.4082   -0.8165
%        -0.8165    0.4082    0.4082
% 
% 
%     b =
% 
%         0.4082
%         0.4082
%         0.4082
% 
% 
%     Aeq =
% 
%         0.5774    0.5774    0.5774
% 
% 
%     beq =
% 
%         0.5774


  %%initial stuff
  
    if nargin<2, tol=1e-10; end

    [M,N]=size(V);

    
    if M==1
      A=[];b=[];
      Aeq=eye(N); beq=V(:);
      return
    end
    
    

    
    
    p=V(1,:).';
    X=bsxfun(@minus,V.',p);
    
    
    %In the following, we need Q to be full column rank 
    %and we prefer E compact.
    
    if M>N  %X is wide
        
     [Q, R, E] = qr(X,0);  %economy-QR ensures that E is compact.
                           %Q automatically full column rank since X wide
                           
    else%X is tall, hence non-solid polytope
        
     [Q, R, P]=qr(X);  %non-economy-QR so that Q is full-column rank.
     
     [~,E]=max(P);  %No way to get E compact. This is the alternative. 
        clear P
    end
    
    
   diagr = abs(diag(R));

    
   if nnz(diagr)    
       
        %Rank estimation
        r = find(diagr >= tol*diagr(1), 1, 'last'); %rank estimation
    
    
        iE=1:length(E);
        iE(E)=iE;

       
       
        Rsub=R(1:r,iE).';

        if r>1

          [A,b]=vert2con(Rsub,tol);
         
        elseif r==1
            
           A=[1;-1];
           b=[max(Rsub);-min(Rsub)];

        end

        A=A*Q(:,1:r).';
        b=bsxfun(@plus,b,A*p);
        
        if r<N
         Aeq=Q(:,r+1:end).';      
         beq=Aeq*p;
        else
           Aeq=[];
           beq=[];
        end

   else %Rank=0. All points are identical
      
       A=[]; b=[];
       Aeq=eye(N);
       beq=p;
       

   end
   
   
%            ibeq=abs(beq);
%             ibeq(~beq)=1;
%            
%            Aeq=bsxfun(@rdivide,Aeq,ibeq);
%            beq=beq./ibeq;
   
           
           
function [A,b] = vert2con(V,tol)
% VERT2CON - convert a set of points to the set of inequality constraints
%            which most tightly contain the points; i.e., create
%            constraints to bound the convex hull of the given points
%
% [A,b] = vert2con(V)
%
% V = a set of points, each ROW of which is one point
% A,b = a set of constraints such that A*x <= b defines
%       the region of space enclosing the convex hull of
%       the given points
%
% For n dimensions:
% V = p x n matrix (p vertices, n dimensions)
% A = m x n matrix (m constraints, n dimensions)
% b = m x 1 vector (m constraints)
%
% NOTES: (1) In higher dimensions, duplicate constraints can
%            appear. This program detects duplicates at up to 6
%            digits of precision, then returns the unique constraints.
%        (2) See companion function CON2VERT.
%        (3) ver 1.0: initial version, June 2005.
%        (4) ver 1.1: enhanced redundancy checks, July 2005
%        (5) Written by Michael Kleder, 
%
%Modified by Matt Jacobson - March 29,2011
% 


k = convhulln(V);
c = mean(V(unique(k),:));


V = bsxfun(@minus,V,c);
A  = nan(size(k,1),size(V,2));

dim=size(V,2);
ee=ones(size(k,2),1);
rc=0;

for ix = 1:size(k,1)
    F = V(k(ix,:),:);
    if lindep(F,tol) == dim
        rc=rc+1;
        A(rc,:)=F\ee;
    end
end

A=A(1:rc,:);
b=ones(size(A,1),1);
b=b+A*c';

% eliminate duplicate constraints:
[A,b]=rownormalize(A,b);
[discard,I]=unique( round([A,b]*1e6),'rows');

A=A(I,:); % NOTE: rounding is NOT done for actual returned results
b=b(I);
return


      
 function [A,b]=rownormalize(A,b)
 %Modifies A,b data pair so that norm of rows of A is either 0 or 1
 
  if isempty(A), return; end
 
  normsA=sqrt(sum(A.^2,2));
  idx=normsA>0;
  A(idx,:)=bsxfun(@rdivide,A(idx,:),normsA(idx));
  b(idx)=b(idx)./normsA(idx);       
                  
 
 function [r,idx,Xsub]=lindep(X,tol)
%Extract a linearly independent set of columns of a given matrix X
%
%    [r,idx,Xsub]=lindep(X)
%
%in:
%
%  X: The given input matrix
%  tol: A rank estimation tolerance. Default=1e-10
%
%out:
%
% r: rank estimate
% idx:  Indices (into X) of linearly independent columns
% Xsub: Extracted linearly independent columns of X

   if ~nnz(X) %X has no non-zeros and hence no independent columns
       
       Xsub=[]; idx=[];
       return
   end

   if nargin<2, tol=1e-10; end
   

           
     [Q, R, E] = qr(X,0); 
     
     diagr = abs(diag(R));


     %Rank estimation
     r = find(diagr >= tol*diagr(1), 1, 'last'); %rank estimation

     if nargout>1
      idx=sort(E(1:r));
        idx=idx(:);
     end
     
     
     if nargout>2
      Xsub=X(:,idx);                      
     end     