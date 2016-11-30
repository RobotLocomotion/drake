function path = gazeboModelPath(model)
% Finds a model:// within the environment variable GAZEBO_MODEL_PATH.
% Unlike ROSPATH, I *think* the logic here is very simple (see 
% gazebo/src/gazebo/common/SystemPaths.cc): models must actually appear
% directly in the model path (no recursive searching, etc).

persistent model_db;

if isempty(model_db) % cache db 
  model_db = containers.Map('KeyType','char','ValueType','char');
  model_db = [model_db;searchdirs(getenv('GAZEBO_MODEL_PATH'))];
end

if model_db.isKey(model)
  path = model_db(model);
else
  % todo: check local dir?
  error('couldn''t find GAZEBO model %s.\n GAZEBO_MODEL_PATH=%s\n',model,getenv('GAZEBO_MODEL_PATH'));
end
  
end


function model_db = searchdirs(pathstr)

model_db = containers.Map('KeyType','char','ValueType','char');
while ~isempty(pathstr)
  [token,pathstr]=strtok(pathstr,pathsep);
  d = dir(token);
  for i=find([d.isdir])
    if (d(i).name(1)=='.') continue; end
    model_db(d(i).name)=fullfile(token,filesep,d(i).name);
  end
end

end

