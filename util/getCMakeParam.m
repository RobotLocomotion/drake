function val = getCMakeParam(param)

[retval,val] = system(['cmake -L -N ',fullfile(getDrakePath,'pod-build'),' | grep ', param,' | cut -d "=" -f2']);
val = strtrim(val);

end
