function m=staticMethods(classname)
% return all of the public static methods from a class 
% (skipping abstract and hidden methods)


%% this works fine, but might not be robust to versions
%m1=methods(classname);
%m2=methods(classname,'-full');
%static_ind = strmatch('Static',m2);
%m = m1(static_ind);

%% a better? way
c=meta.class.fromName(classname);
m=c.Methods;
b=cellfun(@(a) a.Static && ~a.Hidden && ~a.Abstract && strcmp(a.Access,'public'),m);
m=cellfun(@(a) a.Name,m(b),'UniformOutput',false);
