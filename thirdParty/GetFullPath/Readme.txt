Installation:

Copy the files to a folder in your Matlab path.

Windows: Compile the C-file:

  >> InstallMex GetFullPath.c uTest_GetFullPath

  You can compile and start the unit-test manually also:

  >> mex -O GetFullPath.c
  >> uTest_GetFullPath

  When you do not have a compiler, download pre-compiled
  binaries instead:  http://www.n-simon.de/mex

Jan Simon, 17-Jan-2013 00:50