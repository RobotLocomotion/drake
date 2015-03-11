/*-------------------------------------------------------

                        Spruce
            Filesystem Wrapper Library for C++11

    Copyright (c) 2014, Joseph Davis (@josephdavisco)
    All rights reserved. See LICENSE for license info.

---------------------------------------------------------*/
#include "spruce.hh"

#include <utility>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdio>
extern "C" {
    #include <sys/stat.h>
}
/*-------------------------------------------------------

    Windows includes

---------------------------------------------------------
Notes:

POSIX functions named such as mkdir are deprecated on
Windows. Functions are prefixed with an underscore (_),
mkdir() becomes _mkdir().

"In Windows NT, both the (\) and (/) are valid path
delimiters in character strings in run-time routines."

http://msdn.microsoft.com/en-us/library/

---------------------------------------------------------*/
#if defined(_WIN32) || defined(_WIN64)
#define SPRUCE_WIN
#include <direct.h>
auto SPRUCE_CHDIR  = _chdir;
auto SPRUCE_GETCWD = _getcwd;
auto SPRUCE_RMDIR  = _rmdir;
auto SPRUCE_MKDIR  = _mkdir;
#else
/*-------------------------------------------------------

    Unix/Posix includes

---------------------------------------------------------*/
extern "C" {
    #include <sys/types.h>
    #include <unistd.h>
}
auto SPRUCE_CHDIR  = chdir;
auto SPRUCE_GETCWD = getcwd;
auto SPRUCE_RMDIR  = rmdir;
auto SPRUCE_MKDIR  = []( const char* pathTomake )
{
    return mkdir( pathTomake, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
};
#endif

#ifndef S_ISDIR
#define S_ISDIR(mode)  (((mode) & S_IFMT) == S_IFDIR)
#endif

#ifndef S_ISREG
#define S_ISREG(mode)  (((mode) & S_IFMT) == S_IFREG)
#endif

/*-------------------------------------------------------
    class path default constructor
---------------------------------------------------------*/
spruce::path::path()
{
    path_str = "";
}

/*-------------------------------------------------------
    class path constructor
---------------------------------------------------------*/
spruce::path::path( std::string path_ ) : path_str( path_ )
{
    normalize();
}

/*-------------------------------------------------------
    class path destructor
---------------------------------------------------------*/
spruce::path::~path()
{
}

/*-------------------------------------------------------
    path.setStr
---------------------------------------------------------*/
void spruce::path::setStr( std::string path_ )
{
    path_str = path_;
    normalize();
}

/*-------------------------------------------------------
    path.getStr
---------------------------------------------------------*/
std::string spruce::path::getStr() const
{
    return path_str;
}

/*-------------------------------------------------------
    class path.split
---------------------------------------------------------*/
std::vector<std::string> spruce::path::split()
{
    std::vector<std::string> v;
    std::istringstream ss( path_str );
    while ( !ss.eof() )
    {
        std::string temp;
        std::getline( ss, temp, '/' );
        v.push_back( temp );
    }
    return v;
}

/*-------------------------------------------------------
    class path.extension
---------------------------------------------------------*/
std::string spruce::path::extension()
{
    size_t dot( path_str.rfind( '.' ) );

    if ( dot == std::string::npos )
    {
        return "";
    }

    return path_str.substr( dot );
}

void spruce::path::append(std::string const & str)
{
  if(isFile())
    return;
  std::string str_append;

  if(str.find('/') == 0)
    str_append = str.substr(1, str.length());
  else
    str_append = str;

  if(path_str.rfind('/') == path_str.length())
    path_str.append(str_append);
  else
    path_str.append("/" + str_append);
}

void spruce::path::setExtension(std::string const & str)
{
  size_t ext = path_str.rfind(extension());
  if(ext == std::string::npos)
    ext = path_str.length();
  path_str = path_str.substr(0, ext) + str;
}

/*-------------------------------------------------------
    class path.root
---------------------------------------------------------*/
std::string spruce::path::root()
{
    size_t position( path_str.rfind( '/' ) );

    if ( position == std::string::npos )
    {
        return "";
    }

    return path_str.substr( 0, position );
}

/*-------------------------------------------------------
    class path.isFile
---------------------------------------------------------*/
bool spruce::path::isFile() const
{
    if ( !exists() ) return false;

#ifdef SPRUCE_WIN

    struct _stat attributes;
    _stat( path_str.c_str(), &attributes );

#else

    struct stat attributes;
    stat( path_str.c_str(), &attributes );

#endif

    return S_ISREG( attributes.st_mode ) != 0;
}

/*-------------------------------------------------------
    class path.isDir
---------------------------------------------------------*/
bool spruce::path::isDir() const
{
    if ( !exists() ) return false;

#ifdef SPRUCE_WIN

    struct _stat attributes;
    _stat( path_str.c_str(), &attributes );

#else

    struct stat attributes;
    stat( path_str.c_str(), &attributes );

#endif

    return S_ISDIR( attributes.st_mode ) != 0;
}

/*-------------------------------------------------------
    class path.exists
---------------------------------------------------------*/
bool spruce::path::exists() const
{
#ifdef SPRUCE_WIN
    struct _stat attributes;
    return _stat( path_str.c_str(), &attributes ) == 0;
#else
    struct stat attributes;
    return stat( path_str.c_str(), &attributes ) == 0;
#endif
}

/*-------------------------------------------------------
    class path.setAsHome
---------------------------------------------------------*/
void spruce::path::setAsHome()
{
    char* home;
#ifdef SPRUCE_WIN
    home = std::getenv( "HOMEPATH" );
#else
    home = std::getenv( "HOME" );
#endif
    if ( home != NULL )
    {
        setStr( home );
    }
}

/*-------------------------------------------------------
    class path.setAsTemp
---------------------------------------------------------*/
void spruce::path::setAsTemp()
{
    char* cwd = std::getenv( "TMPDIR" );
    if ( !cwd ) cwd = std::getenv( "TEMPDIR" );
    if ( !cwd ) cwd = std::getenv( "TMP" );
    if ( !cwd ) cwd = std::getenv( "TEMP" );
    if ( !cwd )
    {
        setStr( "/tmp" );
    }
    else
    {
        setStr( cwd );
    }
}

/*-------------------------------------------------------
    class path.setAsCurrent
---------------------------------------------------------*/
void spruce::path::setAsCurrent()
{
    setStr( SPRUCE_GETCWD( NULL, 0 ) );
}

/*-------------------------------------------------------
    class path.normalize
---------------------------------------------------------*/
void spruce::path::normalize()
{
    std::replace( path_str.begin(), path_str.end(), '\\', '/' );

    if ( path_str.back() == '/' ) path_str = path_str.substr(0, path_str.length()-1);
}

/*-------------------------------------------------------
    ostream operator<< class path
---------------------------------------------------------*/
std::ostream& operator<<( std::ostream& output, const spruce::path& p )
{
    return output << p.getStr();
}

/*-------------------------------------------------------
    file::remove
---------------------------------------------------------*/
bool spruce::file::remove( const spruce::path& p )
{
    if ( !p.isFile() || ( std::remove( ( p.getStr() ).c_str() ) != 0 ) )
        return false;
    return true;
}

/*-------------------------------------------------------
    file::rename
---------------------------------------------------------*/
bool spruce::file::rename( const spruce::path& source, const spruce::path& dest )
{
    if ( std::rename( source.getStr().c_str(), dest.getStr().c_str() ) != 0 )
        return false;
    return true;
}

/*-------------------------------------------------------
    file::copy
---------------------------------------------------------*/
bool spruce::file::copy( const spruce::path& source, const spruce::path& dest )
{
    std::ifstream in( source.getStr(), std::ios::binary );
    std::ofstream out( dest.getStr(), std::ios::binary );

    out << in.rdbuf();
    if ( out.fail() )
    {
        return false;
    }
    return true;
}

/*-------------------------------------------------------
    file::readAsString
---------------------------------------------------------*/
bool spruce::file::readAsString( const spruce::path& p,
                                 std::string& readTo )
{

    std::ifstream file( p.getStr() );
    std::stringstream ss;

    if ( file.is_open() )
    {
        while ( !file.eof() )
        {
            ss << static_cast<char>( file.get() );
        }
        file.close();
    }
    else
    {
        return false;
    }

    readTo = ss.str();

    return true;
}

/*-------------------------------------------------------
    file::writeAsString
---------------------------------------------------------*/
bool spruce::file::writeAsString( const spruce::path& p,
                                  const std::string& content, bool append )
{

    std::ofstream out;

    if ( append )
    {
        out.open( ( p.getStr() ).c_str(),
                  std::ios_base::out | std::ios_base::app );
    }

    else
    {
        out.open( ( p.getStr() ).c_str() );
    }

    if ( out.fail() )
    {
        return false;
    }

    out << content;

    out.close();

    return true;
}

/*-------------------------------------------------------
    dir::mkdir
---------------------------------------------------------*/
bool spruce::dir::mkdir( const spruce::path& p )
{
    if ( SPRUCE_MKDIR( ( p.getStr() ).c_str() ) != 0 )
    {
        return false;
    }
    return true;
}

/*-------------------------------------------------------
    dir::mkdirAll
---------------------------------------------------------*/
bool spruce::dir::mkdirAll( const spruce::path& p )
{
    size_t pos = 1;
    std::string temp( p.getStr() );

    if ( temp == "" ) return false;

    while ( ( pos = temp.find( '/', pos + 1 ) ) != std::string::npos )
    {
        if ( ( path( temp.substr( 0, pos ) ) ).exists() ) continue;
        // if making a directory on the path fails we cannot continue
        if ( SPRUCE_MKDIR( ( temp.substr( 0, pos ) ).c_str() ) != 0 )
            return false;
    }

    if ( SPRUCE_MKDIR( temp.c_str() ) != 0 ) return false;

    return true;
}

/*-------------------------------------------------------
    dir::rmdir
---------------------------------------------------------*/
bool spruce::dir::rmdir( const spruce::path& p )
{
    if ( SPRUCE_RMDIR( ( p.getStr() ).c_str() ) != 0 )
    {
        return false;
    }
    return true;
}

/*-------------------------------------------------------
    dir::rename
---------------------------------------------------------*/
bool spruce::dir::rename( const spruce::path& source, const spruce::path& dest )
{
    // cstdio accepts the name of a file or directory
    return spruce::file::rename( source, dest );
}

/*-------------------------------------------------------
    dir::chdir
---------------------------------------------------------*/
bool spruce::dir::chdir( const spruce::path& p )
{
    if ( SPRUCE_CHDIR( ( p.getStr() ).c_str() ) != 0 ) return false;
    return true;
}

