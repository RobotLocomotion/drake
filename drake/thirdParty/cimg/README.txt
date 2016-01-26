--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
                            ____  _   _  ____
                           (_  _)( )_( )( ___)
                             )(   ) _ (  )__)
                            (__) (_) (_)(____)
    ___  ____  __  __  ___     __    ____  ____  ____    __    ____  _  _
   / __)(_  _)(  \/  )/ __)   (  )  (_  _)(  _ \(  _ \  /__\  (  _ \( \/ )
  ( (__  _)(_  )    (( (_-.    )(__  _)(_  ) _ < )   / /(__)\  )   / \  /
   \___)(____)(_/\/\_)\___/   (____)(____)(____/(_)\_)(__)(__)(_)\_) (__)


                    C++ Template Image Processing Toolkit

                             ( http://cimg.eu )

                                   _cimg_version

--------------------------------------------------------------------------------

# Summary
#---------

  The CImg Library is a small, open-source, modern C++ toolkit for image processing.
  It consists in a single header file 'CImg.h' providing a minimal set of C++
  classes and methods that can be used in your own sources, to load/save,
  process and display images. Very portable (Unix/X11,Windows, MacOS X, FreeBSD, .. ),
  efficient, easy to use, it's a pleasant library for developping image processing
  algorithms in C++.

# Authors and contributors :
#----------------------------

  - David Tschumperle (project leader) ( http://tschumperle.users.greyc.fr/ )

  - Maksim Aizenshtein
  - Alberto Albiol
  - Antonio Albiol
  - Neil Brown
  - Haz-Edine Assemlal
  - Vincent Barra
  - Wolf Blecher
  - Romain Blei
  - Yohan Bentolila
  - Jerome Boulanger
  - Pierre Buyssens
  - Sebastien Coudert
  - Frederic Devernay
  - Olivier D'Hondt
  - Francois-Xavier Dupe
  - Gerd von Egidy
  - Eric Fausett
  - Jean-Marie Favreau
  - Sebastien Fourey
  - Alexandre Fournier
  - Hon-Kwok Fung
  - Vincent Garcia
  - David Grimbichler
  - Jinwei Gu
  - Jean-Daniel Guyot
  - Matt Hanson
  - Sebastien Hanel
  - Michael Holroyd
  - Christoph Hormann
  - Werner Jainek
  - Daniel Kondermann
  - Pierre Kornprobst
  - Jan W. Krieger
  - Orges Leka
  - Francois Lauze
  - Xie Long
  - Thomas Martin
  - Cesar Martinez
  - Jean Martinot
  - Arnold Meijster (Center for High Performance Computing and Visualization, University of Groningen/The Netherlands)
  - Nikita Melnichenko
  - Julien Morat
  - Baptiste Mougel
  - Jovana Milutinovich
  - Guillaume Nee
  - Adam Newgas
  - Francisco Oliveira
  - Andrea Onofri
  - Renaud Peteri
  - Martin Petricek
  - Paolo Prete
  - Adrien Reboisson
  - Klaus Schneider
  - Jakob Schluttig
  - Veronique Souchaud
  - Konstantin Spirin
  - David G. Starkweather
  - Rainer Steffens
  - Grzegorz Szwoch
  - Thierry Thomas
  - Yu-En-Yun
  - Vo Duc Khanh
  - Phillip Wood
  - Bug Zhao
  - Haibo Zheng

# Institution
#-------------

 GREYC Image / CNRS UMR 6072 / FRANCE

 The CImg Library project started in 2000, at the INRIA-Sophia
 Antipolis/France ( http://www-sop.inria.fr/ ), in the ROBOTVIS / ODYSSEE Team.
 Since October 2004, it is maintained and developed in the Image team of
 the GREYC Lab (CNRS, UMR 6072), in Caen/France.
 Team web page : http://www.greyc.fr/image

# Licenses
#----------

 The source code of the CImg Library is distributed under
 two distinct licenses :

 - The main library file 'CImg.h' is *dual-licensed* :
   It can be either distributed under the CeCILL-C or CeCILL license.
   (see files 'Licence_CeCILL-C_V1-en.txt' and 'Licence_CeCILL_V2-en.txt').
   Both are Free-Software licenses :

     * CeCILL-C is adapted to the distribution of
       library components, and is close in its terms to the well known GNU LGPL license
       (the 'CImg.h' file can thus be used in closed-source products under certain
       conditions, please read carefully the license file).

     * CeCILL is close to (and even compatible with) the GNU GPL license.

 - Most of the other files are distributed under the CeCiLL license
   (file 'Licence_CeCILL_V2-en.txt'). See each file header to see what license applies.

 These two CeCiLL licenses ( http://www.cecill.info/index.en.html ) have been
 created under the supervision of the three biggest research institutions on
 computer sciences in France :

   - CNRS  ( http://www.cnrs.fr/ )
   - CEA   ( http://www.cea.fr/ )
   - INRIA ( http://www.inria.fr/ )

 You have to RESPECT these licenses. More particularly, please carefully read
 the license terms before using the CImg library in commercial products.

# Package structure :
#--------------------

  The main package directory CImg/ is organized as follows :

  - README.txt                 : This file.
  - Licence_CeCILL-C_V1-en.txt : A copy of the CeCiLL-C license file.
  - Licence_CeCILL_V2-en.txt   : A copy of the CeCiLL license.
  - CImg.h                     : The single header file that constitutes the library itself.
  - examples/                  : A directory containing a lot of example programs performing
                                 various things, using the CImg library.
  - html/                      : A directory containing a copy of the CImg web page in html
                                 format. The reference documentation is generated
              		         automatically with the tool 'doxygen' (http://www.doxygen.org).
  - resources/                 : A directory containing some resources files for compiling
                                 CImg examples or packages with various C++ compilers and OS.
  - plugins/                   : A directory containing CImg plug-ins files that can be used to
                                 add specific extra functionalities to the CImg library.

# Getting started
#-----------------

  If you are new to CImg, you should first try to compile the different examples
  provided in the 'examples/' directory, to see what CImg is capable of
  (as CImg is a template-based library, no prior compilation of the library is mandatory).
  Look at the 'resources/' directory to ease this compilation on different platforms.

  Then, you can look at the documentation 'html/reference/' to learn more about CImg
  functions and classes. Finally, you can participate to the 'Forum' section
  of the CImg web page and ask for help if needed.

# End of file
#------------
