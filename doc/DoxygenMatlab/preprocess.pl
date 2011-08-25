#!/usr/bin/perl -w

sub preprocess
{

    my $fname=$_[0];
    open(my $in, $fname);

    my $fileoutname = $fname . ".preprocess.m";

    # set to 1 to use <pre> above and below in places where there are not @ commands and the comments are long
    my $usePre = 0;

    my $incommentblock = 0;
    my $blankLineCheck = 0;
    my $functionDef = "";
    my $commentString = "";
    my $output = "";
    my $numCommentLines = 0;
    my $firstCommentLine = 0;
    my $firstCommentLineEndsNum = -1;

    my $firstAt = -1;
    my $lastAt = -1;

    my $optionStartString = "%> <dl>\n%> <dt>\n%> <b>\n%> Options:\n%> </b>\n%> </dt>\n%> <dd>\n%> <table border=\"0\" cellpadding=\"0\" cellspacing=\"2\">";

    my $optionString = $optionStartString;

    my $optionEndString = "\n%> </table>\n%> </dd>\n%> </dl>\n";

    my $haveOption = 0;
    my $lastWasOption = 0;
    my $lastWasBlank = 0;
    my $functionReturnVals = "";
    my $functionDefsReturns = 0;

    while (<$in>)
    {
        # default varable ($_) reads a line at a time
        
        #print "line: ".$_;
        
        if ($incommentblock == 1)
        {
            # check to see if this line defines return values
            if ((/\@retval/))
            {
                $functionDefsReturns = 1;
            }
            
            
            if ((/^\s*%/))
            {
                #print ("this is a comment right after a function!\n");
                
                # since we're seeing a comment, we don't need to worry about skipping a line for the comment block
                $blankLineCheck = 0;

                
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
                
                # check to see if this is an @option that we are parsing ourselves
                if ($thisComment =~ m/^[\s%>]*\@option[s]? ([^ ]*) (.*)/)
                {
                    
                    # read this option as @option <option_name> <option description (might be long)>
                    #
                    # ex: @option t_max               - longest trajectory (default 10s)
                    
                    $thisOptionName = $1;
                    $thisOptionDesc = $2;
                    
                    if ($lastWasOption == 1)
                    {
                        $optionString = $optionString . "\n%> </td>\n%> </tr>";
                    }
                    
                    $optionString = $optionString . "\n%> <tr>\n%> <td valign=\"top\">\n%> <em>" . $thisOptionName . "</em>&nbsp;\n%> </td>\n%> <td valign=\"top\">" . $thisOptionDesc;
                    
                    $haveOption = 1;
                    $lastWasOption = 1;
                    
                    
                } else {

                    if ($lastWasOption == 1 && $lastWasBlank == 1 && $thisComment !~ m/^[\s%>]*[\n\r]/)
                    {
                        # put option down since this is text that isn't an option anymore
                        
                        $optionString = $optionString . "\n%> </td>\n%> </tr>";
                        
                        $commentString = $commentString . $optionString . $optionEndString;
                        
                        $lastWasOption = 0;
                        $haveOption = 0;
                        $optionString = $optionStartString;
                        
                        $commentString = $commentString . $thisComment;
                        
                    } elsif ($lastWasOption == 1) {
                        # this is a line-wrapped option (not blank, but not @option)
                        
                        # strip out the comment
                      #  $thisComment =~ s/%>//;
                        
                        $optionString = $optionString . "\n" . $thisComment;
                    } else {
                        # end stuff dealing with @option

                    
                        if ($firstCommentLineEndsNum < 0 && $thisComment =~ m/[^%>\s]/)
                        {
                            $firstCommentLineEndsNum = length($commentString . $thisComment);
                        }
                        
                        $commentString = $commentString . $thisComment;
                        
                        $numCommentLines = $numCommentLines + 1;
                    } #end if
                }
                
                if ($thisComment =~ m/^[\s%>]*[\n\r]/)
                {
                    $lastWasBlank = 1;
                } else {
                    $lastWasBlank = 0;
                }
                
            } elsif ($blankLineCheck == 1) {
                # we are in a comment block, but didn't see a comment on the first line.
                $blankLineCheck = 0;
                
            } else {
            
            
            
                # this line isn't a comment (just left a comment block)
                
                
                
                $incommentblock = 0;
                
                if ($haveOption == 1)
                {
                    $commentString = $commentString . $optionString . $optionEndString;
                }
                
                # check to see if the comment ever defined return values
                # if not, do it for the user
                if ($functionDefsReturns == 0 && $functionReturnVals =~ m/\w/)
                {
                    # this function doesn't comment the return values, so do it for the user
                    
                    # the list ends when we find a "="
                    # and replace each comma with a new line and a @retval
                    
                    $functionReturnVals =~ s/,/\n%> \@retval /g;
                    
                    $functionReturnVals =~ s/[\[\]=]//g;
                    
                    $functionReturnVals = "\n%> \@retval " . $functionReturnVals . "\n";
                    
                    #print ">>>> " . $functionReturnVals . "\n";
                    $commentString = $commentString . $functionReturnVals;
                    
                }
                
                #print("not searching for comments right after functions anymore!\n");
    ################## uncomment to use <pre> ##############
                # swap the order of the comments and the function definitions
                # the last $_ adds this line to the ouptut, since we should include it in the normal
                # order of things
    #            if ($numCommentLines > 2 && $usePre == 1)
    #            {
    #                # since this is a long comment, we'll want to consider use preformatting everywhere we don't have
    #                # @param, or other @commands
    #                
    #                # add a <pre> to the first line
    #                
    #                #$commentString = "%> <pre>\n%> " . $commentString;
    #                
    #                if ($firstAt > 0 && $lastAt > 0)
    #                {
    #                    $firstCommentLine = substr $commentString, 0, $firstCommentLineEndsNum;
    #                    $commentStringBeforeAt = substr $commentString, $firstCommentLineEndsNum, ($firstAt - $firstCommentLineEndsNum);
    #                    $commentStringBetweenAt = substr $commentString, $firstAt, ($lastAt - $firstAt);
    #                    $commentStringAfterAt = substr $commentString, $lastAt;
    #                    
    #                    
    #                    $commentString = $firstCommentLine . "%> \n%> <pre>\n%> \n" . $commentStringBeforeAt . "%> \n%> </pre>\n%> \n" . $commentStringBetweenAt . "%> \n%> <pre>\n%> \n" . $commentStringAfterAt . "%> \n%> </pre>\n%> \n";
    #                    
    #                } else {
    #                    $firstCommentLine = substr $commentString, 0, $firstCommentLineEndsNum;
    #                    $commentStringRest = substr $commentString, $firstCommentLineEndsNum;
    #                    $commentString = $firstCommentLine . "%> \n%> <pre>\n%> \n" . $commentStringRest . "%> \n%> </pre>\n%> \n";
    #                
    #                }
    #            }
    ########################################################

                $output = $output . $commentString;
                
                $output = $output . $functionDef . $_;
                
                $commentString = "";
                $numCommentLines = 0;
                $firstCommentLine = "";
                $firstAt = -1;
                $lastAt = -1;
                $firstCommentLineEndsNum = -1;
                $optionString = $optionStartString;
                $lastWasOption = 0;
                $haveOption = 0;
                $lastWasBlank = 0;
                $functionReturnVals = "";
                $functionDefsReturns = 0;
            }

        } elsif ((/(^\s*function)\s*([\] \w\d,_\[]+=)?\s*([.\w\d_-]*)\s*\(?([\w\d\s,~]*)\)?(%?.*)/) || (/(^\s*classdef)\s*(\s*\([\{\}\?\w,=\s]+\s*\))?\s*([\w\d_]+)\s*<?\s*([\s\w\d._&]+)?(.*)/))
        {
            #print "that was a function!\n";
            $functionDef = $_;
            
            $incommentblock = 1;
            $blankLineCheck = 1; # check for a single blank line before the comment block
            
            $functionReturnVals = $2;
            
        } else {
            # nothing special happening... just move along
            $output = $output . $_;
        }
    }
    close($in);
    
    #print "--------------\n";
    
    open(MYFILE, ">" . $fileoutname);
    
    print MYFILE $output;

    close(MYFILE);
    
    return $fileoutname;
}

1;
