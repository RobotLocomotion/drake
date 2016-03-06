function addpath_pods
% does the pod thing - looking up to 4 directories above for a folder named
% build (with /matlab) and then adds it to the path

pfx='';
for i=1:4
  if exist(fullfile(pfx,'build','matlab'),'file')
    disp(['Adding ', fullfile(pwd,pfx,'build','matlab'), ' to the matlab path']);
    addpath(fullfile(pwd,pfx,'build','matlab'));
    break;
  end
  pfx = fullfile('..',pfx);
end
