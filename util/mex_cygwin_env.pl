#!/usr/bin/perl

# Sets up the required environment variables from the mexopts.bat file (converting the appropriate ones to cygwin format)
# Usage:
#   eval `mex_cygwin_env.pl`

$MEXOPTS = `touch dummy.c && mex.bat -v dummy.c 2> /dev/null`; system("rm dummy.c");
die "Can't find your mex options file. Make sure mex.bat is in your path (MEX = $MEX).  MATLAB >= R2014a is not supported by this script yet.\n" if ($MEXOPTS eq "");

if ($MEXOPTS =~ m/Set PATH/) { # Then it's the new mex interface, MATLAB version >= 2014a 

    @lines = split(/\n/,$MEXOPTS);

    @lines = grep(/Set /,@lines);
    foreach $line (@lines) {
	$line =~ s/Set //g;
	($var,$val) = split(/=/,$line);
	chomp($var); chomp($val);
	$var =~ s/[^a-zA-Z]//g;
	$val =~ s/^\s//g;
	$val =~ s/[^a-zA-Z0-9\\\/ \(\).:;-]//g;  # only let these characters through (had some issue with special characters)
	if ($var =~ m/^PATH/) {
	    $newpath = $val;
#	    print("PATH = $newpath\n");
        } else { 
	    $ENV{$var} = "$val";
#            system("echo $var = \$$var\n");
	}
    }
} else {
    @lines = split(/\n/,$MEXOPTS);
    @MEXOPTS = grep(/Options file/,@lines);
    $MEXOPTS_FILE = $MEXOPTS[0];
    $MEXOPTS_FILE =~ s/[^=]*=\s*//; chomp($MEXOPTS_FILE);

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
	if ($var =~ m/^PATH/) {
	    $newpath = $val;
        } else { 
	    $ENV{$var} = "$val";  # set the environment variable (only for this process and children) for future references to that var
#            system("echo $var = \$$var\n");
	}
	}
    }

    close(MEXOPTS)
}

@vars_to_export_with_cygpath = ('PATH');
@vars_to_export_with_original_path = ('INCLUDE','LIB','LIBPATH');
foreach $var (@vars_to_export_with_cygpath) {
    $valuelist = "";
    if ($var =~ m/PATH/) {
#	print("PATH = $newpath\n");
	@values = split(/;/,$newpath);
#	$valuelist = "\$PATH";
    } else {
	@values = split(/;/,$ENV{$var});
    }
    foreach $val (@values) {
#        system("echo cygpath \"$val\"");
	$val = `cygpath \"$val\"`;
        chomp $val;
	if ($valuelist eq "") {
	    $valuelist = $val;
	} else {
	    $valuelist = $valuelist . ":" . $val;
	}
    }
    print "export $var=\"$valuelist\"\n";
}
foreach $var (@vars_to_export_with_original_path) {
    print "export $var=\"$ENV{$var}\"\n";
}

