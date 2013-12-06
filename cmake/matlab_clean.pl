#!/usr/bin/perl -w

# This script:
#  - Runs matlab with all input arguments passed through
#  - Captures the output from stdout and stderr and the return value of the matlab process
#  - Filters the output to remove the non-standard characters
#  - Prints the output, and exits with the return value of matlab 
# This appears to be a necessary evil to get clean matlab output to ctest:  
#  http://www.mail-archive.com/cmake@cmake.org/msg02175.html

my $cmd = 'matlab';
foreach my $a(@ARGV) {
  $cmd .= " \"$a\"";
}
$cmd .= ' 2>&1 ';

# use a pipe so as not to buffer output. in case matlab takes a vacation midstream
open(my $command_stream, "-|", $cmd);
while(<$command_stream>){
    # le junque. on separate lines for readability
    $_ =~ s/.*exclude an item from Time Machine.*\n//g;
    $_ =~ s/[^\x{0009}\x{000a}\x{000d}\x{0020}-\x{D7FF}\x{E000}-\x{FFFD}]+//g;
    $_ =~ s/\x1b\[\?1h//g;
    $_ =~ s/\x1b=//g;
    $_ =~ s/\x1b\[\?1l\x1b>//g;
    print;
}
my $retval = $? >> 8;
exit($retval);

