function spot_install_manifest

oldpath = path;
cd spotless;
spot_install;
newpath = path;
cd ..;

pathdiff = setdiff(regexp(newpath,':','split'),regexp(oldpath,':','split'));

fptr = fopen('spot_install_manifest.txt','w');
fprintf(fptr,'%s\n',pathdiff{:});
fclose(fptr);
