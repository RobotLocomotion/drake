#!/usr/bin/perl

# Sets up the required environment variables from the mexopts.bat file (converting the appropriate ones to cygwin format)
# Usage:
#   eval `mex_cygwin_env.pl`
# or 
#   eval `mex_cygwin_env.pl /cygdrive/c/Users/russt/AppData/Roaming/MathWorks/MATLAB/R2013b/mexopts.bat`
#
# todo: handle MATLAB >= R2014a

if (@ARGV > 0) {
    $MEXOPTS_FILE = $ARGV[0];
} else {
    # recover it automatically
    $MEX = `which mex.bat`; chomp($MEX);
    $MEX = `cygpath -w "$MEX"`;  chomp($MEX); $MEX = "cmd.exe /c \"$MEX\"";  # seems to be more robust (otherwise, doesn't work in .bashrc during ssh login)
    $MEXOPTS_FILE = `$MEX -v | grep "Options file"`;
    $MEXOPTS_FILE =~ s/[^=]*=\s*//; chomp($MEXOPTS_FILE);
    #print "$MEXOPTS_FILE\n";
    die "Can't find your mex options file. Make sure mex.bat is in your path (MEX = $MEX).  MATLAB >= R2014a is not supported by this script yet.\n" if ($MEXOPTS_FILE eq "");
}

open(MEXOPTS,$MEXOPTS_FILE);

while (<MEXOPTS>) {
    chomp;
    if (m/^set/) {
	s/^set\s*//g;                   # zap set from start of line
	s/%([^%]+)%/\$\1/g;             # replace %VAR% with $VAR
	($var,$val) = split /=/;        
	$val =~ s/\"/\\\"/g;            # " -> \"
	$val = `echo "$val"`;           # use the shell to evaluate environment variables in the string
	$val =~ s/[^a-zA-Z0-9\\\/ \(\).:;-]//g;  # only let these characters through (had some issue with special characters)
	$ENV{$var} = "$val";            # set the environment variable (only for this process and children) for future references to that var
#	system("echo $var = \$$var\n");
    }
}

@vars_to_export_with_cygpath = ('PATH');
@vars_to_export_with_original_path = ('INCLUDE','LIB');
foreach $var (@vars_to_export_with_cygpath) {
    $val = `cygpath \"$ENV{$var}\"`;
    chomp $val;
    $val =~ s/;/:/g;
    print "export $var=\"$val\"\n";
}
foreach $var (@vars_to_export_with_original_path) {
    print "export $var=\"$ENV{$var}\"\n";
}

close(MEXOPTS)
