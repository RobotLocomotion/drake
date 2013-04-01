// URDF exceptions
#ifndef _URDF_EXCEPTIONS_HH_
#define _URDF_EXCEPTIONS_HH_
#include<string>
#include<exception>

namespace urdf{

  class ParseError: public std::exception
  {
    public:
      ParseError() {};
      ParseError(const std::string &error_msg): error_msg_(error_msg) {};
      virtual const char* what() const throw()
      {
        return this->error_msg_.c_str();
      };
      ParseError* addMessage( const std::string &msg )
      {
        this->error_msg_ += "\n"+msg;
        return this;
      };
      virtual ~ParseError() throw() {};
    protected:
      std::string error_msg_;
  };
}
#endif
