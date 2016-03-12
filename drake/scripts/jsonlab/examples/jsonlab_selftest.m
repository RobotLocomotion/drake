%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Regression Test Unit of loadjson and savejson
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:4
    fname=sprintf('example%d.json',i);
    if(exist(fname,'file')==0) break; end
    fprintf(1,'===============================================\n>> %s\n',fname);
    json=savejson('data',loadjson(fname));
    fprintf(1,'%s\n',json);
    data=loadjson(json);
end

for i=1:4
    fname=sprintf('example%d.json',i);
    if(exist(fname,'file')==0) break; end
    fprintf(1,'===============================================\n>> %s\n',fname);
    json=saveubjson('data',loadjson(fname));
    fprintf(1,'%s\n',json);
    data=loadubjson(json);
    savejson('',data)
end
