#!/usr/bin/perl -w

require "doc/DoxygenMatlab/preprocess.pl";

if ($#ARGV != 0)
{
  die "Argument must contain filename $#ARGV"
}
else
{
  $fname=$ARGV[0];
}

# If we have a .m file inside a (@)-folder with the same name :
# we will read each file of this folder
if ($fname =~ /^(.*)\@([\d\w\-_]*)[\/\\](\2)\.m/)
{
  $name = $2;
  $nameExt = $name.".m";
  $dir = $1."@".$name."/\*.m";
  @fic = glob($dir);
  $i = 0;
  $listeFic[0] = $fname;
  foreach $my_test (@fic)
  {
    if (!($my_test =~ $nameExt))
    {
      $i++;
      $listeFic[$i] = $my_test;
    }
  }

}
# otherwise @-folder, but .m with a different name : ignore it (we read it when we read the main class file)
elsif ($fname =~ /^(.*)\@([\d\w\-_]*)[\/\\](.*)\.m/)
{
}
# otherwise
else
{
  $listeFic[0] = $fname;
}
$output = "";
foreach $my_fic (@listeFic)
{
  $my_fic2 = preprocess($my_fic);
  open(my $in, $my_fic2);

  $declTypeDef="";
  $inClass = 0;
  $inAbstractMethodBlock = 0;
  $listeProperties = 0;
  $listeEnumeration = 0;
#  $inComment = 0;
  $listeEvents = 0;

  #default to public methods since if we're in a file that is not the main class file
  # we might end up not hitting a method block first, in which case the functions should be public
  $methodAttribute = "public:";


  while (<$in>)
  {

    # convert matlab sytle comments to C style comments
    if (/(^\s*)(%>)(.*)/)
    {
      $output=$output."$1///$3";
    }

    #if inside a property block, we'll take all the comments, not just ones pointed
    # explicitly at us (ue take % and %> comments)
    # also do this for an abstract method block
    if ( $listeProperties == 1 )
    {
        if  ((/(^\s*)(%)(.*)/))
        {
            $output=$output."$1///$3";
        }
    }

    if ( ($inAbstractMethodBlock == 1 && (/(^\s*)(%)(.*)/))  )
    {

      $output=$output."$1///$3";
    }

    # check for matlab end's
    if (($listeProperties == 1) && (/(^\s*\bend\b\s*)/))
    {
      $listeProperties = 0;
    }
    if (($inAbstractMethodBlock == 1) && (/(^\s*\bend\b\s*)/))
    {
      $inAbstractMethodBlock = 0;
    }

    if (($listeProperties == 1) && (/^\s*([\w\d]*)\s*(=\s*[\w\d{}'',\s\[\]\(\)\.]*)?[;\s]*(%[>]?.*)?/))
    {
      $propertyName = $1;
      if (/^\s*([\w\d]*)\s*(=\s*[\w\d{}'',\s\[\]\(\)\.]*)[;\s]*(%[>]?.*)/) {
	$propertyValue = $2;
        $propertyComment = $3;
      } else {
	$propertyValue = "";
        $propertyComment = "";
      }

      if (!($propertyName =~ /^$/))
      {
        if ($typeProperties =~ /Constant/)
        {
          $properties = $propertyName."$propertyValue;";
        }
        else
        {
          $properties = $propertyName.";";
        }

        # replace all "%>"s and "%"s with "//"
        $properties =~ s/%>/\/\/\//g;
        $properties =~ s/%/\/\/\//g;
        $propertyComment =~ s/%>/\/\/\//;
        $propertyComment =~ s/%/\/\/\//;
        $output=$output.$propertyComment . "\n" . $typeProperties."Property ".$properties;
      }
    }
    if (($listeEnumeration == 1) && (/(^\s*\bend\b\s*)/))
    {
      $listeEnumeration = 0;
      $output=$output."};";
    }
    if (($listeEvents == 1) && (/(^\s*\bend\b\s*)/))
    {
      $listeEvents = 0;
      $output=$output."};";
    }
    if (($listeEvents == 1) && (/^\s*([\w\d]*)\s*/))
    {
      $name_event = $1;
      if (!($name_event =~ /^$/))
      {
        $event = $name_event.",";
        $event =~ s/%>/\/\/\//g;
        $event =~ s/%/\/\//g;
        $output=$output.$event;
      }
    }
    if (($listeEnumeration == 1) && (/^\s*([\w\d]*)\s*(\(.*\))?(%>.*)?/))
    {
      $name_enum = $1;
      $val_enum = $2;
      if (!($name_enum =~ /^$/))
      {
        if (!($val_enum =~ /^$/))
        {
          $enum = "$name_enum=$val_enum,";
          $enum =~ s/%>/\/\/\//g;
          $enum =~ s/%/\/\//g;
          $output=$output.$enum;
        }
        else
        {
          $enum = "$name_enum,";
          $enum =~ s/%>/\/\/\//g;
          $enum =~ s/%/\/\//g;
          $output=$output.$enum;
        }
      }
    }
    if (/(^\s*function)\s*([\] \w\d,_\[]+=)?\s*([.\w\d_-]*)\s*\(?([\w\d\s,~=]*)\)?(%?.*)/)
    {
      $functionKeyWord = $1;
      $functionName = $3;
      $arguments = $4;
#      $linecomment = $5;
      if ($inClass == 0)
      {
        $output = $declTypeDef.$output;
        $declTypeDef = "";
      }
      $arguments =~ s/,/,/g;
      $arguments =~ s/~/ignoredArg/g;
      $arguments = "$arguments";

      if ($arguments =~ /^$/)
      {
        $arguments = "";
      }
      $ligne = "$methodAttribute $functionKeyWord $functionName($arguments);";
      $output=$output.$ligne;
    }
    # Signature of functions in abstract methods
    elsif ((/^\s*([\] \w\d,_\[]+=)?\s*([.\w\d_-]+)\s*\(?([\w\d\s,~]*)\)?(%?.*)/) & ($inAbstractMethodBlock == 1) )
    {
      $functionName = $2;
      $arguments = $3;
      $arguments =~ s/,/,/g;
      $arguments =~ s/~/ignoredArg/g;
      $arguments = "$arguments";
      if ($arguments =~ /^$/)
      {
        $arguments = "";
      }
      $ligne = "$methodAttribute function $functionName($arguments);";
      $output=$output.$ligne;
    }
    # inheritance for classes
    if (/(^\s*classdef)\s*(\s*\([\{\}\?\w,=\s]+\s*\))?\s*([\w\d_]+)\s*<?\s*([\s\w\d._&]+)?(.*)/)
    {
      $className = $3;
      if (/(^\s*classdef)\s*(\s*\([\{\}\?\w,=\s]+\s*\))?\s*([\w\d_]+)\s*<\s*([\s\w\d._&]+)(.*)/) {
	$classInheritance = $4;
        $classInheritance =~ s/&/,public /g;
        $classDef = "class ".$className.":public $classInheritance";
      }
      else
      {
        $classDef = "class ".$className;
      }
      $output=$output."/// \@nosubgrouping\n".$classDef;
      $output=$output."{";
      $output=$output.$declTypeDef;
      $output=$output."public:\n";
      $inClass = 1;
    }
    if (/(^\s*properties)\s*(\s*\([\w,=\s]+\s*\))?(.*)/)
    {
      $listeProperties = 1;
      $typeProperties = "public:\n";
      if (/(^\s*properties)\s*(\s*\([\w,=\s]+\s*\))(.*)/) {
	$propertiesAttributes = $2;
	if (lc($propertiesAttributes) =~ /(access\s*=\s*private)/)
	  {
	    $typeProperties = "private:\n"
	  }
	elsif (lc($propertiesAttributes) =~ /(access\s*=\s*public)/)
	  {
	    $typeProperties = "public:\n"
	  }
	elsif (lc($propertiesAttributes) =~ /(access\s*=\s*protected)/)
	  {
	    $typeProperties = "protected:\n"
	  }
	if ((lc($propertiesAttributes) =~ /(constant\s*=\s*false)/) || (lc($propertiesAttributes) =~ /(~constant)/))
	  {
	  }
	elsif (lc($propertiesAttributes) =~ /(constant(\s*=\s*true\s*)?)/)
	  {
	    $typeProperties = $typeProperties." Constant ";
	  }
      }
    }
    if (/(^\s*enumeration)\s*(.*)/)
    {
      $listeEnumeration = 1;
      $output=$output."public:\nenum ".$className." {";
    }
    if (/(^\s*events)\s*(.*)/)
    {
      $listeEvents = 1;
      $output=$output."public:\nenum Events {";
    }
    if (/(^\s*methods)\s*(.*)/)
    {
      $methodAttribute = "public:\n";
      if (/^\s*methods\s*(\([\w,=\s]+\s*\)).*/)
      {
	$methodsAttributes = $1;
	if (lc($methodsAttributes) =~ /(access\s*=\s*private)/)
	  {
	    $methodAttribute = "private:\n"
	  }
	elsif (lc($methodsAttributes) =~ /(access\s*=\s*protected)/)
	  {
	    $methodAttribute = "protected:\n"
	  }
	elsif (lc($methodsAttributes) =~ /(access\s*=\s*public)/)
	  {
	    $methodAttribute = "public:\n"
	  }
	if (lc($methodsAttributes) =~ /(abstract(\s*=\s*true\s*)?)/)
	  {
	    $inAbstractMethodBlock = 1;
	    $methodAttribute = $methodAttribute." virtual ";
	  }
	if ((lc($methodsAttributes) =~ /(static\s*=\s*false)/) || (lc($methodsAttributes) =~ /(~static)/))
	  {
	  }
	elsif (lc($methodsAttributes) =~ /(static(\s*=\s*true\s*)?)/)
	  {
	    $methodAttribute = $methodAttribute." static";
	  }
      }
    }
    $output=$output."\n";
  }
  close $in;

  # delete the preprocess file
  unlink($my_fic2);

}
$output=$output."};\n";
#print $output;



# now move each comment block above the previous function block so that we can support matlab's commenting that is by default inside functions

# match a comment block and then a non-comment block


#$output =~ s/\/\/\//ZZZ/g;

#if ($output =~ m/(?-s)\s*(\/\/\/.*(?s)(.*?))\/\/\//g)
#if ($output =~ m/(?s)(.*?)(?-s)(\/\/\/.*?)/)
#{
#    print "\n1-------------\n";
##    $output2 = $1;
#    print $1;
#    print "\n2------------\n";
#    print $2;
#    print "\n3------------\n";
#    print $3;
#    print "\n--------------\n";
#}

print $output;
