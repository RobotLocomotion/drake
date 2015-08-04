function plotregion(A,b,lb,ub,c,transp,points,linetyp,start_end)
% The function plotregion plots closed convex regions in 2D/3D. The region
% is formed by the matrix A and the vectors lb and ub such that Ax>=b
% and lb<=x<=ub, where the region is the set of all feasible x (in R^2 or R^3).
% An option is to plot points in the same plot.
%
% Usage:   plotregion(A,b,lb,ub,c,transp,points,linetyp,start_end)
%
% Input:
%
% A - matrix. Set A to [] if no A exists
% b - vector. Set b to [] if no b exists
% lb - (optional) vector. Set lb to [] if no lb exists
% ub - (optional) vector. Set ub to [] if no ub exists
% c - (optional) color, example 'r' or [0.2 0.1 0.8], {'r','y','b'}.
%       Default is a random colour.
% transp - (optional) is a measure of how transparent the
%           region will be. Must be between 0 and 1. Default is 0.5.
% points - (optional) points in matrix form. (See example)
% linetyp - (optional) How the points will be marked.
%           Default is 'k-'.
% start_end - (optional) If a special marking for the first and last point
%              is needed.
%
% If several regions must be plotted A ,b, lb, ub and c can be stored as a cell array {}
% containing all sub-set information (see example1.m and example3.m). 
%
% Written by Per Bergstr�m 2006-01-16


if nargin<2
    error('Too few arguements for plotregion');
elseif nargin<6
    transp=0.5;
end

if iscell(A)
    
    if isempty(A{1})
        m=0;
        n=max(length(lb),length(ub));
    else
        [m,n]=size(A{1});
    end
    [lll,p]=size(A);
    for i=1:p
        if size(b{i},1)==1
            b{i}=b{i}';
        end
        
        if nargin>2
            if not(isempty(lb))
                if not(isempty(lb{i}))
                    if size(lb{i},1)==1
                        lb{i}=lb{i}';
                    end
                    A{i}=[A{i};eye(n)];
                    b{i}=[b{i};lb{i}];
                end
            end
        end
        if nargin>3
            if not(isempty(ub))
                if not(isempty(ub{i}))
                    if size(ub{i},1)==1
                        ub{i}=ub{i}';
                    end
                    A{i}=[A{i};-eye(n)];
                    b{i}=[b{i};-ub{i}];
                end
            end
        end
    end
    if nargin<5
        c=cell(1,p);
        for i=1:p
            c{i}=[rand rand rand];
        end
    end
    
    if nargin>=5
        if isempty(c);
            c=cell(1,p);
            for i=1:p
                c{i}=[rand rand rand];
            end
        else
            for i=1:p
                if isempty(c{i});
                    c{i}=[rand rand rand];
                end
            end
        end
    end
    
    
    warning off
    if n==2
        eq=cell(1,p);
        X=cell(1,p);
        for pp=1:p
            eq{pp}=zeros(2,1);
            X{pp}=zeros(2,1);
            [m,n]=size(A{pp});
            
            for i=1:(m-1)
                for j=(i+1):m
                    try
                        x=A{pp}([i j],:)\b{pp}([i j]);
                        if and(min((A{pp}*x-b{pp}))>-1e-6,min((A{pp}*x-b{pp}))<Inf)
                            X{pp}=[X{pp},x];
                            eq{pp}=[eq{pp},[i j]'];
                        end
                    end
                end
            end
        end
        
        xmi=min(X{1}(1,2:end));
        xma=max(X{1}(1,2:end));
        ymi=min(X{1}(2,2:end));
        yma=max(X{1}(2,2:end));
        
        for pp=2:p
            xmi=min(min(X{pp}(1,2:end)),xmi);
            xma=max(max(X{pp}(1,2:end)),xma);
            ymi=min(min(X{pp}(2,2:end)),ymi);
            yma=max(max(X{pp}(2,2:end)),yma);
        end
        
        if nargin>=7
            xmi2=min(points(1,:));
            xma2=max(points(1,:));
            ymi2=min(points(2,:));
            yma2=max(points(2,:));
            
            xmi=min(xmi,xmi2);
            xma=max(xma,xma2);
            ymi=min(ymi,ymi2);
            yma=max(yma,yma2);
        end
        
        axis([(xmi-0.1) (xma+0.1) (ymi-0.1) (yma+0.1)]);
        
        if nargin==7
            plot(points(1,:)',points(2,:)','k-');hold on;
        elseif nargin==8
            plot(points(1,:)',points(2,:)',linetyp);hold on;
        elseif nargin==9
            plot(points(1,:)',points(2,:)',linetyp);hold on;
            if length(start_end)==2
                plot(points(1,1)',points(2,1)',start_end);hold on;
                plot(points(1,end)',points(2,end)',start_end);hold on;
            elseif length(start_end)==4
                plot(points(1,1)',points(2,1)',start_end(1:2));hold on;
                plot(points(1,end)',points(2,end)',start_end(3:4));hold on;
            end
        end
        
        for pp=1:p
            [rad,col]=size(X{pp});
            xm=mean(X{pp}(:,2:end),2);
            Xdiff=X{pp}(:,2:end);
            
            for j=1:(col-1)
                Xdiff(:,j)=Xdiff(:,j)-xm;
                Xdiff(:,j)=Xdiff(:,j)/norm(Xdiff(:,j));
            end
            costhe=zeros((col-1),1);
            
            for j=1:(col-1)
                costhe(j)=Xdiff(:,1)'*Xdiff(:,j);
            end
            
            [cc,ind]=min(abs(costhe));
            ref2=Xdiff(:,ind(1))-(Xdiff(:,ind(1))'*Xdiff(:,1))*Xdiff(:,1);
            ref2=ref2'/norm(ref2);
            
            for j=1:(col-1)
                if ref2*Xdiff(:,j)<0
                    costhe(j)=-2-costhe(j);
                end
            end
            [sooo,ind3]=sort(costhe);
            set(patch(X{pp}(1,ind3+1)',X{pp}(2,ind3+1)',c{pp}),'FaceAlpha',transp);
        end
        
    elseif n==3
        eq=cell(1,p);
        X=cell(1,p);
        for pp=1:p
            eq{pp}=zeros(3,1);
            X{pp}=zeros(3,1);
            [m,n]=size(A{pp});
            
            for i=1:(m-2)
                for j=(i+1):(m-1)
                    for k=(j+1):m
                        try
                            x=A{pp}([i j k],:)\b{pp}([i j k]);
                            if and(min((A{pp}*x-b{pp}))>-1e-6,min((A{pp}*x-b{pp}))<Inf)
                                X{pp}=[X{pp},x];
                                eq{pp}=[eq{pp},[i j k]'];
                            end
                            
                        end
                    end
                end
            end
        end
        
        xmi=min(X{1}(1,2:end));
        xma=max(X{1}(1,2:end));
        ymi=min(X{1}(2,2:end));
        yma=max(X{1}(2,2:end));
        zmi=min(X{1}(3,2:end));
        zma=max(X{1}(3,2:end));
        
        for pp=2:p
            xmi=min(min(X{pp}(1,2:end)),xmi);
            xma=max(max(X{pp}(1,2:end)),xma);
            ymi=min(min(X{pp}(2,2:end)),ymi);
            yma=max(max(X{pp}(2,2:end)),yma);
            zmi=min(min(X{pp}(3,2:end)),zmi);
            zma=max(max(X{pp}(3,2:end)),zma);
        end
        
        if nargin>=7
            xmi2=min(points(1,:));
            xma2=max(points(1,:));
            ymi2=min(points(2,:));
            yma2=max(points(2,:));
            zmi2=min(points(3,:));
            zma2=max(points(3,:));
            
            xmi=min(xmi,xmi2);
            xma=max(xma,xma2);
            ymi=min(ymi,ymi2);
            yma=max(yma,yma2);
            zmi=min(zmi,zmi2);
            zma=max(zma,zma2);
        end
        
        axis([(xmi-0.1) (xma+0.1) (ymi-0.1) (yma+0.1) (zmi-0.1) (zma+0.1)]);
        
        if nargin==7
            plot3(points(1,:)',points(2,:)',points(3,:)','k-');hold on;
        elseif nargin==8
            plot3(points(1,:)',points(2,:)',points(3,:)',linetyp);hold on;
        elseif nargin==9
            plot3(points(1,:)',points(2,:)',points(3,:)',linetyp);hold on;
            if length(start_end)==2
                plot3(points(1,1)',points(2,1)',points(3,1)',start_end);hold on;
                plot3(points(1,end)',points(2,end)',points(3,end)',start_end);hold on;
            elseif length(start_end)==4
                plot3(points(1,1)',points(2,1)',points(3,1)',start_end(1:2));hold on;
                plot3(points(1,end)',points(2,end)',points(3,end)',start_end(3:4));hold on;
            end
        end
        
        for pp=1:p
            [m,n]=size(A{pp});
            for i=1:m
                [ind1,ind2]=find(eq{pp}==i);
                lind2=length(ind2);
                if lind2>0
                    xm=mean(X{pp}(:,ind2),2);
                    Xdiff=X{pp}(:,ind2);
                    for j=1:lind2
                        Xdiff(:,j)=Xdiff(:,j)-xm;
                        Xdiff(:,j)=Xdiff(:,j)/norm(Xdiff(:,j));
                    end
                    costhe=zeros(lind2,1);
                    
                    for j=1:lind2
                        costhe(j)=Xdiff(:,1)'*Xdiff(:,j);
                    end
                    
                    [cc,ind]=min(abs(costhe));
                    ref2=Xdiff(:,ind(1))-(Xdiff(:,ind(1))'*Xdiff(:,1))*Xdiff(:,1);
                    ref2=ref2'/norm(ref2);
                    
                    for j=1:lind2
                        if ref2*Xdiff(:,j)<0
                            costhe(j)=-2-costhe(j);
                        end
                    end
                    [sooo,ind3]=sort(costhe);
                    set(patch(X{pp}(1,ind2(ind3))',X{pp}(2,ind2(ind3))',X{pp}(3,ind2(ind3))',c{pp}),'FaceAlpha',transp);
                end
            end
            
        end
    else
        error('not 2D or 3D');
    end
    
else      %  A is not a cell
    
    if isempty(A)
        m=0;
        n=max(length(lb),length(ub));
    else
        [m,n]=size(A);
    end
    
    if size(b,1)==1
        b=b';
    end
    
    if nargin>2
        if not(isempty(lb))
            if size(lb,1)==1
                lb=lb';
            end
            A=[A;eye(n)];
            b=[b;lb];
            m=m+n;
        end
    end
    if nargin>3
        if not(isempty(ub))
            if size(ub,1)==1
                ub=ub';
            end
            
            A=[A;-eye(n)];
            b=[b;-ub];
            m=m+n;
        end
    end
    
    if nargin<5
        c=[rand rand rand];
    end
    
    if nargin>=5
        if isempty(c);
            c=[rand rand rand];
        end
    end
    
    warning off
    if n==2
        eq=zeros(2,1);
        X=zeros(2,1);
        for i=1:(m-1)
            for j=(i+1):m
                try
                    x=A([i j],:)\b([i j]);
                    if and(min((A*x-b))>-1e-6,min((A*x-b))<Inf)
                        X=[X,x];
                        eq=[eq,[i j]'];
                    end
                end
            end
        end
        
        xmi=min(X(1,2:end));
        xma=max(X(1,2:end));
        ymi=min(X(2,2:end));
        yma=max(X(2,2:end));
        
        if nargin>=7
            xmi2=min(points(1,:));
            xma2=max(points(1,:));
            ymi2=min(points(2,:));
            yma2=max(points(2,:));
            
            xmi=min(xmi,xmi2);
            xma=max(xma,xma2);
            ymi=min(ymi,ymi2);
            yma=max(yma,yma2);
        end
        axis([(xmi-0.1) (xma+0.1) (ymi-0.1) (yma+0.1)]);
        
        if nargin==7
            plot(points(1,:)',points(2,:)','k-');hold on;
        elseif nargin==8
            plot(points(1,:)',points(2,:)',linetyp);hold on;
        elseif nargin==9
            plot(points(1,:)',points(2,:)',linetyp);hold on;
            if length(start_end)==2
                plot(points(1,1)',points(2,1)',start_end);hold on;
                plot(points(1,end)',points(2,end)',start_end);hold on;
            elseif length(start_end)==4
                plot(points(1,1)',points(2,1)',start_end(1:2));hold on;
                plot(points(1,end)',points(2,end)',start_end(3:4));hold on;
            end
        end
        
        [rad,col]=size(X);
        xm=mean(X(:,2:end),2);
        Xdiff=X(:,2:end);
        
        for j=1:(col-1)
            Xdiff(:,j)=Xdiff(:,j)-xm;
            Xdiff(:,j)=Xdiff(:,j)/norm(Xdiff(:,j));
        end
        costhe=zeros((col-1),1);
        
        for j=1:(col-1)
            costhe(j)=Xdiff(:,1)'*Xdiff(:,j);
        end
        
        [cc,ind]=min(abs(costhe));
        ref2=Xdiff(:,ind(1))-(Xdiff(:,ind(1))'*Xdiff(:,1))*Xdiff(:,1);
        ref2=ref2'/norm(ref2);
        
        for j=1:(col-1)
            if ref2*Xdiff(:,j)<0
                costhe(j)=-2-costhe(j);
            end
        end
        [sooo,ind3]=sort(costhe);
        set(patch(X(1,ind3+1)',X(2,ind3+1)',c),'FaceAlpha',transp);
        
    elseif n==3
        eq=zeros(3,1);
        X=zeros(3,1);
        
        for i=1:(m-2)
            for j=(i+1):(m-1)
                for k=(j+1):m
                    try
                        x=A([i j k],:)\b([i j k]);
                        if and(min((A*x-b))>-1e-6,min((A*x-b))<Inf)
                            X=[X,x];
                            eq=[eq,[i j k]'];
                        end
                        
                    end
                end
            end
        end
        xmi=min(X(1,2:end));
        xma=max(X(1,2:end));
        ymi=min(X(2,2:end));
        yma=max(X(2,2:end));
        zmi=min(X(3,2:end));
        zma=max(X(3,2:end));
        
        if nargin>=7
            xmi2=min(points(1,:));
            xma2=max(points(1,:));
            ymi2=min(points(2,:));
            yma2=max(points(2,:));
            zmi2=min(points(3,:));
            zma2=max(points(3,:));
            
            xmi=min(xmi,xmi2);
            xma=max(xma,xma2);
            ymi=min(ymi,ymi2);
            yma=max(yma,yma2);
            zmi=min(zmi,zmi2);
            zma=max(zma,zma2);
        end
        
        axis([(xmi-0.1) (xma+0.1) (ymi-0.1) (yma+0.1) (zmi-0.1) (zma+0.1)]);
        
        if nargin==7
            plot3(points(1,:)',points(2,:)',points(3,:)','k-');hold on;
        elseif nargin==8
            plot3(points(1,:)',points(2,:)',points(3,:)',linetyp);hold on;
        elseif nargin==9
            plot3(points(1,:)',points(2,:)',points(3,:)',linetyp);hold on;
            if length(start_end)==2
                plot3(points(1,1)',points(2,1)',points(3,1)',start_end);hold on;
                plot3(points(1,end)',points(2,end)',points(3,end)',start_end);hold on;
            elseif length(start_end)==4
                plot3(points(1,1)',points(2,1)',points(3,1)',start_end(1:2));hold on;
                plot3(points(1,end)',points(2,end)',points(3,end)',start_end(3:4));hold on;
            end
        end
        
        for i=1:m
            [ind1,ind2]=find(eq==i);
            lind2=length(ind2);
            if lind2>0
                xm=mean(X(:,ind2),2);
                Xdiff=X(:,ind2);
                for j=1:lind2
                    Xdiff(:,j)=Xdiff(:,j)-xm;
                    Xdiff(:,j)=Xdiff(:,j)/norm(Xdiff(:,j));
                end
                costhe=zeros(lind2,1);
                
                for j=1:lind2
                    costhe(j)=Xdiff(:,1)'*Xdiff(:,j);
                end
                
                [cc,ind]=min(abs(costhe));
                ref2=Xdiff(:,ind(1))-(Xdiff(:,ind(1))'*Xdiff(:,1))*Xdiff(:,1);
                ref2=ref2'/norm(ref2);
                
                for j=1:lind2
                    if ref2*Xdiff(:,j)<0
                        costhe(j)=-2-costhe(j);
                    end
                end
                [sooo,ind3]=sort(costhe);
                set(patch(X(1,ind2(ind3))',X(2,ind2(ind3))',X(3,ind2(ind3))',c),'FaceAlpha',transp);
            end
        end
    else
        error('Your region must be in 2D or 3D!');
    end
    
end
warning on
