#!/usr/bin/perl -w

# This script:
#  - Runs matlab with all input arguments passed through
#  - Captures the output from stdout and stderr and the return value of the matlab process
#  - Filters the output to remove the non-standard characters
#  - Prints the output, and exits with the return value of matlab 
# This appears to be a necessary evil to get clean matlab output to ctest:  
#  http://www.mail-archive.com/cmake@cmake.org/msg02175.html

#use lib "./Text-Unidecode-0.04/lib";
#use Text::Unidecode;

my $cmd = 'matlab';
foreach my $a(@ARGV) {
  $cmd .= " \"$a\"";
}
$cmd .= ' 2>&1 ';

my $matlab_output = `$cmd`;
my $retval = $? >> 8;

$matlab_output =~ s/.*exclude an item from Time Machine.*\n//g;
#$matlab_output = unidecode($matlab_output);  # didn't seem to help
$matlab_output =~ s/[^\x{0009}\x{000a}\x{000d}\x{0020}-\x{D7FF}\x{E000}-\x{FFFD}]+//g;

print($matlab_output);
exit($retval);

