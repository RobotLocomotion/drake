#!/usr/bin/perl -w

if ($#ARGV != 0)
{
  die "Argument must contain filename $#ARGV"
}
else
{
  $fname=$ARGV[0];
}

open(my $in, $fname);

$incommentblock = 0;
$functionDef = "";
$commentString = "";
$output = "";
$numCommentLines = 0;
$firstCommentLine = 0;
$firstCommentLineEndsNum = -1;

$firstAt = -1;
$lastAt = -1;

while (<$in>)
{
    # default varable ($_) reads a line at a time
    
    #print "line: ".$_;
    
    if ($incommentblock == 1)
    {
        if ((/^\s*%/))
        {
            #print ("this is a comment right after a function!\n");
            $thisComment = $_;
            $thisComment =~ s/\s*%/%>/;
            #$thisComment =~ s/[\n\r]/<br>\n/;
            
            
            
            # if the line doesn't have and @ in it, double-break it to make the output look good
            # since the user is likely formatting their comments with line breaks
            
            if ($thisComment =~ m/@/)
            {
                if ($firstAt < 0)
                {
                    $firstAt = length($commentString);
                }
                
                $lastAt = length($commentString . $thisComment);
            }
            #} else {
#                if ($thisComment =~ m/(\s*).*/)
#                {
#                #    $thisComment = $thisComment . $1 . "%>\n";
#                    #$thisComment = $thisComment;
#                }
            #}
            
            if ($firstCommentLineEndsNum < 0 && $thisComment =~ m/[^%>\s]/)
            {
                $firstCommentLineEndsNum = length($commentString . $thisComment);
            }
            
            if ($numCommentLines > 2)
            {
                # we're going to be processing with <pre>'s since the comment is long
                #$thisComment =~ s/%>/%> <pre>/;
                #$thisComment =~ s/[\n\r]/<\/pre>\n/;
                #$thisComment = "%> <pre>" . $thisComment . "</pre>";
                
                # close a pre if we detect and @param or @retval
                
                #if (($thisComment =~ s/\@param/<\/pre>\@param/) || ($thisComment =~ s/\@retval/<\/pre>\@retval/))
                #{
                 #   $lastLineHadCommand = 1;
                    #$thisComment = $thisComment . "%> <pre>\n";
                #}
                
                
                
            }
            
            $commentString = $commentString . $thisComment;
            
            $numCommentLines = $numCommentLines + 1;
        } else {
            $incommentblock = 0;
            #print("not searching for comments right after functions anymore!\n");
            
            # we want to use preformatted text since our comments are formatted directly in matlab source
            # to do this, first strip out the leading tabs (but keep extra tabs)
#            if ($commentString =~ m/(\s*)%>/)
#            {
#                $beginningTabs = $1;
#                print "\n-------\n" . $beginningTabs . "\n------\n";
#                
##                $commentString =~ s/$beginningTabs/\n/g;
#            }
            
            # swap the order of the comments and the function definitions
            # the last $_ adds this line to the ouptut, since we should include it in the normal
            # order of things
            if ($numCommentLines > 2)
            {
                # since this is a long comment, we'll want to consider use preformatting everywhere we don't have
                # @param, or other @commands
                
                # add a <pre> to the first line
                
                #$commentString = "%> <pre>\n%> " . $commentString;
                
                if ($firstAt > 0 && $lastAt > 0)
                {
                    $firstCommentLine = substr $commentString, 0, $firstCommentLineEndsNum;
                    $commentStringBeforeAt = substr $commentString, $firstCommentLineEndsNum, ($firstAt - $firstCommentLineEndsNum);
                    $commentStringBetweenAt = substr $commentString, $firstAt, ($lastAt - $firstAt);
                    $commentStringAfterAt = substr $commentString, $lastAt;
                    
                    
                    $commentString = $firstCommentLine . "%> \n%> <pre>\n%> \n" . $commentStringBeforeAt . "%> \n%> </pre>\n%> \n" . $commentStringBetweenAt . "%> \n%> <pre>\n%> \n" . $commentStringAfterAt . "%> \n%> </pre>\n%> \n";
                    
                } else {
                    $firstCommentLine = substr $commentString, 0, $firstCommentLineEndsNum;
                    $commentStringRest = substr $commentString, $firstCommentLineEndsNum;
                    $commentString = $firstCommentLine . "%> \n%> <pre>\n%> \n" . $commentStringRest . "%> \n%> </pre>\n%> \n";
                
                }
                
                
                # check to see if the person is using "@param", "@retval", or "<pre>" options
                # if not, do it for them
                
                #if ($commentString =~ m/(\@param|\@retval|<pre>)/)
                #{
                    # let the user do their own formatting.
                #} else {
                    #$commentString =~ s/[\n\r]/\n%> <pre>\n/;
                    
                    
                        
                    
                    #$commentString = $commentString . "%> </pre>\n";
    #                $commentString = $firstCommentLine . "%> <pre>\n" . $commentString . "%> </pre>\n";;
                #}
            }

#            $commentString = substr $commentString, 0, -1;
            $output = $output . $commentString;
            
            $output = $output . $functionDef . $_;
            
            $commentString = "";
            $numCommentLines = 0;
            $firstCommentLine = "";
            $firstAt = -1;
            $lastAt = -1;
            $firstCommentLineEndsNum = -1;
        }

    } elsif ((/(^\s*function)\s*([\] \w\d,_\[]+=)?\s*([.\w\d_-]*)\s*\(?([\w\d\s,~]*)\)?(%?.*)/) || (/(^\s*classdef)\s*(\s*\([\{\}\?\w,=\s]+\s*\))?\s*([\w\d_]+)\s*<?\s*([\s\w\d._&]+)?(.*)/))
    {
        #print "that was a function!\n";
        $functionDef = $_;
        
        $incommentblock = 1;
    } else {
        # nothing special happening... just move along
        $output = $output . $_;
    }
}

#print "--------------\n";
print $output;
