function filename = getFullPathFromRelativePath(filename)
% it is not safe to reference a file using a relative path; if the m-file
% you're calling is in the matlab path then it could be called from
% anywhere.  this method attempts to address that issue by changing 
% a relative

mystack = dbstack;
path_to_mfile_which_called_this_method = fileparts(which(mystack(2).file));
filename=GetFullPath(fullfile(path_to_mfile_which_called_this_method,filename));