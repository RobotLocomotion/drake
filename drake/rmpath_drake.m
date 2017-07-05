function rmpath_drake

root = getDrakePath();

if ispc
  p = getenv('PATH');
  warning('Drake lib directories have been added to your system path and are not removed (yet)');
  % todo: remove added paths
end

rmpath(fullfile(root,'thirdParty','bsd','GetFullPath'));
rmpath(fullfile(root,'thirdParty','bsd','polytopes'));
rmpath(fullfile(root,'matlab','util'));
rmpath(fullfile(root,'matlab','util','geometry'));
rmpath(fullfile(root,'matlab','util','visualization'));

javarmpath(fullfile(drake_get_base_path,'share','java','drake.jar'));
javarmpath(fullfile(drake_get_base_path,'share','java','lcmtypes_drake.jar'));
