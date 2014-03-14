function str = msspoly2str(p)
% str =  msspoly2str(p) - Returns a string representing an msspoly. This should
% eventually move to spotless. It's essentially their msspoly/display method
% but with sprintf instead of fprintf. Unfortunately, due to the way MATLAB
% treats subsref inside and outside of class definitions, and the particular
% way in which msspoly/subsref is written, several changes needed to be made to
% make this function work from drake/util. 
%

if isempty(p)
    str = sprintf('   Empty msspoly matrix: %d-by-%d\n',p.dim(1),p.dim(2));
    return
end

% REPLACE
S{p.m,p.m}=[];             % strings of entries
% WITH
%S{p.dim(1),p.dim(2)}=[];             % strings of entries
% END REPLACE

% REMOVE
p_sub = p.sub;
p_pow = p.pow;
p_coeff = p.coeff;
p_var = p.var;
% END REMOVE

ns = size(p.sub,1);

for i=1:ns,
    % REPLACE
    ii=p_sub(i,1);
    jj=p_sub(i,2);
    S{ii,jj}=msspoly.term_to_string(S{ii,jj},[p_var(i,:) p_pow(i,:) p_coeff(i,:)]);
    % WITH
    %ii=p.sub(i,1);
    %jj=p.sub(i,2);
    %S{ii,jj}=msspoly.term_to_string(S{ii,jj},[p.var(i,:) p.pow(i,:) p.coeff(i,:)]);
    % END REPLACE
end

% REPLACE
L=zeros(1,p.n);
for i=1:p.n,
% WITH
%L=zeros(1,p.dim(2));
%for i=1:p.dim(2),
% END REPLACE
    L(i)=3;
    % REPLACE
    for j=1:p.m,
    % WITH
    %for j=1:p.dim(1),
    % END REPLACE
        k=length(S{j,i});
        if k==0,
            S{j,i}=' 0';
        else
            L(i)=max(L(i),k);
        end
    end
end
str = [];

% REPLACE
for i=1:p.m,
% WITH
%for i=1:p.dim(1),
% END REPLACE
    if i>1
      str = sprintf('%s\n',str);
    end
    str_i = [];
    % REPLACE
    for j=1:p.n,
    % WITH
    %for j=1:p.dim(2),
    % END REPLACE
        str_i = [str_i,sprintf('%s%s  ',repmat(' ',1,L(j)-length(S{i,j})),S{i,j})];
    end
    str = [str, sprintf('[  %s]',str_i)];
end
        

