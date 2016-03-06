function [V,nr,nre]=lcon2vert(A,b,Aeq,beq,TOL,checkbounds)
%An extension of Michael Kleder's con2vert function, used for finding the 
%vertices of a bounded polyhedron in R^n, given its representation as a set
%of linear constraints. This wrapper extends the capabilities of con2vert to
%also handle cases where the  polyhedron is not solid in R^n, i.e., where the
%polyhedron is defined by both equality and inequality constraints.
% 
%SYNTAX:
%
%  [V,nr,nre]=lcon2vert(A,b,Aeq,beq,TOL)
%
%The rows of the N x n matrix V are a series of N vertices of the polyhedron
%in R^n, defined by the linear constraints
%  
%   A*x  <= b
%   Aeq*x = beq
%
%By default, Aeq=beq=[], implying no equality constraints. The output "nr"
%lists non-redundant inequality constraints, and "nre" lists non-redundant 
%equality constraints.
%
%The optional TOL argument is a tolerance used for both rank-estimation and 
%for testing feasibility of the equality constraints. Default=1e-10. 
%The default can also be obtained by passing TOL=[];
%
%
%EXAMPLE: 
%
%The 3D region defined by x+y+z=1, x>=0, y>=0, z>=0
%is described by the following constraint data.
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
%
%
%  >> V=lcon2vert(A,b,Aeq,beq)
%
%         V =
% 
%             1.0000    0.0000    0.0000
%             0.0000    0.0000    1.0000
%            -0.0000    1.0000    0.0000
%
%




  %%initial argument parsing
  
  nre=[];
  nr=[];
  if nargin<5 || isempty(TOL), TOL=1e-10; end
  if nargin<6, checkbounds=true; end
  
  switch nargin 
      
      case 0
          
           error 'At least 1 input argument required'
       

      case 1
        
         b=[]; Aeq=[]; beq=[]; 
        
          
      case 2
          
          Aeq=[]; beq=[];
          
      case 3
          
          beq=[];
          error 'Since argument Aeq specified, beq must also be specified'
            
  end
  
  
  b=b(:); beq=beq(:);
  
  if xor(isempty(A), isempty(b)) 
     error 'Since argument A specified, b must also be specified'
  end
      
  if xor(isempty(Aeq), isempty(beq)) 
        error 'Since argument Aeq specified, beq must also be specified'
  end
  
  
  nn=max(size(A,2)*~isempty(A),size(Aeq,2)*~isempty(Aeq));
  
  if ~isempty(A) && ~isempty(Aeq) && ( size(A,2)~=nn || size(Aeq,2)~=nn)
      
      error 'A and Aeq must have the same number of columns if both non-empty'
      
  end
  
  
  inequalityConstrained=~isempty(A);  
  equalityConstrained=~isempty(Aeq);

 [A,b]=rownormalize(A,b);
 [Aeq,beq]=rownormalize(Aeq,beq);
 
  if equalityConstrained && nargout>2
 
        
        [discard,nre]=lindep([Aeq,beq].',TOL); 
          
        if ~isempty(nre) %reduce the equality constraints
            
            Aeq=Aeq(nre,:);
            beq=beq(nre);
            
        else    
            equalityConstrained=false;
        end
        
   end
      

  
   %%Find 1 solution to equality constraints within tolerance
  
            
   if equalityConstrained
        
        
       Neq=null(Aeq);   


       x0=pinv(Aeq)*beq;

       if norm(Aeq*x0-beq)>TOL*norm(beq),  %infeasible

          nre=[]; nr=[]; %All constraints redundant for empty polytopes
          V=[]; 
          return;
          
       elseif isempty(Neq)

           V=x0(:).'; 
           nre=(1:nn).'; %Equality constraints determine everything. 
           nr=[];%All inequality constraints are therefore redundant.             
           return
           
       end
 
       rkAeq= nn - size(Neq,2);
       
       
  end  
   
    %%
  if inequalityConstrained && equalityConstrained
     
   AAA=A*Neq;
   bbb=b-A*x0;
    
  elseif inequalityConstrained
      
    AAA=A;
    bbb=b;
   
  elseif equalityConstrained && ~inequalityConstrained
      
       error('Non-bounding constraints detected. (Consider box constraints on variables.)')
      
    
  end
  
  nnn=size(AAA,2);
  

  if nnn==1 %Special case
      
     idxu=sign(AAA)==1;
     idxl=sign(AAA)==-1;
     idx0=sign(AAA)==0;
     
     Q=bbb./AAA;
     U=Q; 
       U(~idxu)=inf;
     L=Q;
       L(~idxl)=-inf;

     
     [ub,uloc]=min(U);
     [lb,lloc]=max(L);
     
     if ~all(bbb(idx0)>=0) || ub<lb %infeasible
         
         V=[]; nr=[]; nre=[];
         return
         
     elseif ~isfinite(ub) || ~isfinite(lb)
         
         error('Non-bounding constraints detected. (Consider box constraints on variables.)')
         
     end
      
     Zt=[lb;ub];
     
     if nargout>1
        nr=unique([lloc,uloc]); nr=nr(:);
     end
     
      
  else    
      
          if nargout>1
           [Zt,nr]=con2vert(AAA,bbb,TOL,checkbounds);
          else
            Zt=con2vert(AAA,bbb,TOL,checkbounds); 
          end
  
  end
  


  if equalityConstrained && ~isempty(Zt)
     
      V=bsxfun(@plus,Zt*Neq.',x0(:).'); 
      
  else
      
      V=Zt;
      
  end
 
  if isempty(V),
     nr=[]; nre=[]; 
  end
  

 function [V,nr] = con2vert(A,b,TOL,checkbounds)
% CON2VERT - convert a convex set of constraint inequalities into the set
%            of vertices at the intersections of those inequalities;i.e.,
%            solve the "vertex enumeration" problem. Additionally,
%            identify redundant entries in the list of inequalities.
% 
% V = con2vert(A,b)
% [V,nr] = con2vert(A,b)
% 
% Converts the polytope (convex polygon, polyhedron, etc.) defined by the
% system of inequalities A*x <= b into a list of vertices V. Each ROW
% of V is a vertex. For n variables:
% A = m x n matrix, where m >= n (m constraints, n variables)
% b = m x 1 vector (m constraints)
% V = p x n matrix (p vertices, n variables)
% nr = list of the rows in A which are NOT redundant constraints
% 
% NOTES: (1) This program employs a primal-dual polytope method.
%        (2) In dimensions higher than 2, redundant vertices can
%            appear using this method. This program detects redundancies
%            at up to 6 digits of precision, then returns the
%            unique vertices.
%        (3) Non-bounding constraints give erroneous results; therefore,
%            the program detects non-bounding constraints and returns
%            an error. You may wish to implement large "box" constraints
%            on your variables if you need to induce bounding. For example,
%            if x is a person's height in feet, the box constraint
%            -1 <= x <= 1000 would be a reasonable choice to induce
%            boundedness, since no possible solution for x would be
%            prohibited by the bounding box.
%        (4) This program requires that the feasible region have some
%            finite extent in all dimensions. For example, the feasible
%            region cannot be a line segment in 2-D space, or a plane
%            in 3-D space.
%        (5) At least two dimensions are required.
%        (6) See companion function VERT2CON.
%        (7) ver 1.0: initial version, June 2005
%        (8) ver 1.1: enhanced redundancy checks, July 2005
%        (9) Written by Michael Kleder
%
%Modified by Matt Jacobson - March 30, 2011
% 


   %%%3/4/2012 Improved boundedness test - unfortunately slower than Michael Kleder's
   if checkbounds
       
    [aa,bb,aaeq,bbeq]=vert2lcon(A,TOL);
    
    if any(bb<=0) || ~isempty(bbeq)
        error('Non-bounding constraints detected. (Consider box constraints on variables.)')
    end
    
    clear aa bb aaeq bbeq
    
   end
 
   dim=size(A,2);
   
   %%%Matt J initialization
   if strictinpoly(b,TOL)  
       
       c=zeros(dim,1);
   
   else
    
            
            slackfun=@(c)b-A*c;

            %Initializer0
            c = pinv(A)*b; %02/17/2012 -replaced with pinv()
            s=slackfun(c);

            if ~approxinpoly(s,TOL) %Initializer1

                c=Initializer1(TOL,A,b,c);
                s=slackfun(c);

            end

            if  ~approxinpoly(s,TOL)  %Attempt refinement

                %disp 'It is unusually difficult to find an interior point of your polytope. This may take some time... '
                %disp ' '   

                c=Initializer2(TOL,A,b,c);
                %[c,fval]=Initializer1(TOL,A,b,c,10000);
                s=slackfun(c);


            end


            if ~approxinpoly(s,TOL)
                    %error('Unable to locate a point near the interior of the feasible region.')
                    V=[];
                    nr=[];
                    return
            end



           if ~strictinpoly(s,TOL) %Added 02/17/2012 to handle initializers too close to polytope surface

                %disp 'Recursing...'


                idx=(  abs(s)<=max(s)*TOL );

                Amod=A; bmod=b; 
                 Amod(idx,:)=[]; 
                 bmod(idx)=[];

                Aeq=A(idx,:); %pick the nearest face to c
                beq=b(idx);


                faceVertices=lcon2vert(Amod,bmod,Aeq,beq,TOL,1);
                if isempty(faceVertices)
                   disp 'Something''s wrong. Couldn''t find face vertices. Possibly polyhedron is unbounded.'
                   keyboard
                end

                c=faceVertices(1,:).';  %Take any vertex - find local recession cone vector
                s=slackfun(c);

                idx=(  abs(s)<=max(s)*TOL );

                Asub=A(idx,:); bsub=b(idx,:);

                [aa,bb,aaeq,bbeq]=vert2lcon(Asub);
                aa=[aa;aaeq;-aaeq];
                bb=[bb;bbeq;-bbeq];

                clear aaeq bbeq


                [bmin,idx]=min(bb);

                 if bmin>=-TOL
                   disp 'Something''s wrong. We should have found a recession vector (bb<0).'
                   keyboard
                 end      



                Aeq2=null(aa(idx,:)).';
                beq2=Aeq2*c;  %find intersection of polytope with line through facet centroid.

                linetips = lcon2vert(A,b,Aeq2,beq2,TOL,1);

                if size(linetips,1)<2
                   disp 'Failed to identify line segment through interior.'
                   disp 'Possibly {x: Aeq*x=beq} has weak intersection with interior({x: Ax<=b}).'
                   keyboard
                end


                lineCentroid=mean(linetips);%Relies on boundedness

                clear aa bb

                c=lineCentroid(:);
                s=slackfun(c);


            end


            b = s;
   end
    %%%end Matt J initialization
    
    
    D=bsxfun(@rdivide,A,b); 
    
    
    k = convhulln(D);
    nr = unique(k(:));
    
    
    
    G  = zeros(size(k,1),dim);
    ee=ones(size(k,2),1);
    discard=false( 1, size(k,1) );
    
    for ix = 1:size(k,1) %02/17/2012 - modified
        
        F = D(k(ix,:),:);
        if lindep(F,TOL)<dim; 
            discard(ix)=1;
            continue; 
        end

        G(ix,:)=F\ee;
        
    end
    
    G(discard,:)=[];
    
    V = bsxfun(@plus, G, c.'); 
    
    [discard,I]=unique( round(V*1e6),'rows');
    V=V(I,:);
    
return


function [c,fval]=Initializer1(TOL, A,b,c,maxIter)
       
    
    
    thresh=-10*max(eps(b));
    
    if nargin>4
     [c,fval]=fminsearch(@(x) max([thresh;A*x-b]), c,optimset('MaxIter',maxIter));
    else
     [c,fval]=fminsearch(@(x) max([thresh;A*x-b]), c); 
    end
    
return          


function c=Initializer2(TOL,A,b,c)
 %norm(  (I-A*pinv(A))*(s-b) )  subj. to s>=0 
  
 
    
    maxIter=100000;
 
    [mm,nn]=size(A);
    
    
    
    
     Ap=pinv(A);        
     Aaug=speye(mm)-A*Ap;
     Aaugt=Aaug.';

    
    M=Aaugt*Aaug;
    C=sum(abs(M),2);
     C(C<=0)=min(C(C>0));
    
    slack=b-A*c;
    slack(slack<0)=0;
    
     
        %     relto=norm(b);
        %     relto =relto + (relto==0); 
        %     
        %      relres=norm(A*c-b)/relto;

     
    IterThresh=maxIter; 
    s=slack; 
    ii=0;
    %for ii=1:maxIter
    while ii<=2*maxIter %HARDCODE
        
       ii=ii+1; 
       if ii>IterThresh, 
           %warning 'This is taking a lot of iterations'
           IterThresh=IterThresh+maxIter;
       end          
          
     s=s-Aaugt*(Aaug*(s-b))./C;   
     s(s<0)=0;

      
       c=Ap*(b-s);
       %slack=b-A*c;
       %relres=norm(slack)/relto;
       %if all(0<slack,1)||relres<1e-6||ii==maxIter, break;  end

       
    end
   
return 




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

     
 function [A,b]=rownormalize(A,b)
 %Modifies A,b data pair so that norm of rows of A is either 0 or 1
 
  if isempty(A), return; end
 
  normsA=sqrt(sum(A.^2,2));
  idx=normsA>0;
  A(idx,:)=bsxfun(@rdivide,A(idx,:),normsA(idx));
  b(idx)=b(idx)./normsA(idx);       
        
 function tf=approxinpoly(s,TOL)
     
     
   smax=max(s);
   
   if smax<=0
      tf=false; return 
   end
   
   tf=all(s>=-smax*TOL);
   
  function tf=strictinpoly(s,TOL)
      
   smax=max(s);
   
   if smax<=0
      tf=false; return 
   end
   
   tf=all(s>=smax*TOL);
   
   
         
         
   
 
     
         
         
         
     