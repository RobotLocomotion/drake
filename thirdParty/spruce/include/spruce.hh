/*-------------------------------------------------------

                        Spruce
            Filesystem Wrapper Library for C++11

    Copyright (c) 2014, Joseph Davis (@josephdavisco)
    All rights reserved. See LICENSE for license info.

---------------------------------------------------------*/
#ifndef SPRUCE_H_
#define SPRUCE_H_

#include <vector>
#include <iostream>
#include <string>

#undef DLLEXPORT_spruce
#if defined(WIN32) || defined(WIN64)
  #if defined(spruce_EXPORTS)
    #define DLLEXPORT_spruce __declspec( dllexport )
  #else
    #define DLLEXPORT_spruce __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT_spruce
#endif

namespace spruce
{

class DLLEXPORT_spruce path
{
    std::string path_str;
public:
    path();

    path( std::string path_ );

    ~path();

    void setStr( std::string path_ );

    std::string getStr() const;

    std::vector<std::string> split();

    std::string extension();

    void setExtension(std::string const & ext);

    void append(std::string const & str);

    std::string root();

    bool isFile() const;

    bool isDir() const;

    bool exists() const;

    void setAsHome();

    void setAsTemp();

    void setAsCurrent();

private:
    void normalize();

    friend std::ostream& operator<<( std::ostream& output, const spruce::path& p );
};

namespace file
{
bool remove( const spruce::path& p );

bool rename( const spruce::path& source, const spruce::path& dest );

bool copy( const spruce::path& source, const spruce::path& dest );

bool readAsString( const spruce::path& p, std::string& readTo );

bool writeAsString( const spruce::path& p, const std::string& content,
                    bool append );
}

namespace dir
{
bool mkdir( const spruce::path& p );

bool mkdirAll( const spruce::path& p );

bool rmdir( const spruce::path& p );

bool rename( const spruce::path& source, const spruce::path& dest );

bool chdir( const spruce::path& p );

}

}

#endif /* SPRUCE_H_ */
