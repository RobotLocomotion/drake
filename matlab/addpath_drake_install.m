function [] = addpath_drake()

root = fileparts(mfilename('fullpath'));
addpath(root);

drake_python_path = fullfile(root, '..', '..', '..', 'lib', 'python2.7', 'site-packages');

if count(py.sys.path, drake_python_path) == 0
    insert(py.sys.path, int32(0), drake_python_path);
end
