function editDrakeConfig(field,val)

load drake_config.mat;
conf = setfield(conf,field,val);
save([conf.root,'/util/drake_config.mat'],'conf');
