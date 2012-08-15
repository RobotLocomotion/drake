function editDrakeConfig(param,val)

load drake_config.mat;
if ~isfield(conf,param)
  error(['I don''t understand the parameter ', param]);
end
conf = setfield(conf,param,val);
save([conf.root,'/util/drake_config.mat'],'conf');

