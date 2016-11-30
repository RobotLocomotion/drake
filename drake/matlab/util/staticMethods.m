function [method,definingclass]=staticMethods(classname)
% return all of the public static methods from a class 
% (skipping abstract and hidden methods)


%% this works fine, but might not be robust to versions
%m1=methods(classname);
%m2=methods(classname,'-full');
%static_ind = strmatch('Static',m2);
%m = m1(static_ind);

%% a better? way
try 
  c=meta.class.fromName(classname);
catch ex
  disp(['parse error in ',classname]);
  rethrow(ex);
end
if isempty(c)
  method={};
  definingclass={};
else
  m=c.Methods;
  b=cellfun(@(a) a.Static && ~a.Hidden && ~a.Abstract && strcmp(a.Access,'public'),m);
  method=cellfun(@(a) a.Name,m(b),'UniformOutput',false);
  definingclass=cellfun(@(a) a.DefiningClass.Name,m(b),'UniformOutput',false);
end