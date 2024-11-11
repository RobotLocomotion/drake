//#####################################################################
// Copyright (c) 2012-2013, Sean Bauer, Eftychios Sifakis.
// Distributed under the FreeBSD license (see license.txt)
//#####################################################################
// Utility classes/functions
//#####################################################################
#include <iostream>
#include <stdexcept>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <SPGrid/Core/SPGrid_Utilities.h>

#ifdef HASWELL
#include <immintrin.h>
#endif

namespace SPGrid{

//#####################################################################
// Functions Bit_Spread/Bit_Pack
//#####################################################################

#ifdef HASWELL
uint64_t Bit_Spread(const ucoord_t data,const uint64_t mask)
{uint64_t uldata=data;return _pdep_u64(uldata,mask);}
uint64_t Bit_Spread(const scoord_t data,const uint64_t mask)
{union{ int64_t sldata; uint64_t uldata; };sldata=data;return _pdep_u64(uldata,mask);}
#endif

// Not well-defined behavior if the packed result is meant to be negative
scoord_t Bit_Pack(const uint64_t data, const uint64_t mask)
{
    union{ int64_t slresult; uint64_t ulresult; };    
#ifdef HASWELL
    ulresult=_pext_u64(data,mask);
#else
    uint64_t uldata=data; int count=0; ulresult=0;
    for(uint64_t bit=1;bit;bit<<=1)
        if(bit & mask) ulresult |= (uldata & bit)>>count; else count++;
#endif
    return (scoord_t)slresult;
}

//#####################################################################
// Function Fatal_Error
//#####################################################################
void Fatal_Error(const char* function,const char* file,int line)
{
    Fatal_Error(function,file,line,"Fatal error");
}
void Fatal_Error(const char* function,const char* file,int line,const char* message)
{
    Fatal_Error(function,file,line,std::string(message));
}
void Fatal_Error(const char* function,const char* file,int line,const std::string& message)
{
    static char buffer[2048];
    sprintf(buffer,"%s:%s:%d: %s",file,function,line,message.c_str());
    std::string error(buffer);
    std::cout<<std::flush;std::cerr<<"\n";
    std::cerr<<"\n*** ERROR: "<<error<<'\n'<<std::endl;
    throw std::runtime_error(error);
}

//#####################################################################
// Function Check_Compliance
//#####################################################################
void Check_Compliance()
{
    if(sysconf(_SC_PAGESIZE)!=4096) FATAL_ERROR("Page size different than 4KB detected");
    if(sizeof(unsigned long)!=8) FATAL_ERROR("unsigned long is not 64-bit integer");
    if(sizeof(size_t)!=8) FATAL_ERROR("size_t is not 64-bit long");
    if(sizeof(void*)!=8) FATAL_ERROR("void* is not 64-bit long");
    typedef enum {dummy=0xffffffffffffffffUL} Long_Enum;
    if(sizeof(Long_Enum)!=8) FATAL_ERROR("Missing support for 64-bit enums");
}

//#####################################################################
// Function Raw_Allocate
//#####################################################################
void* Raw_Allocate(const size_t size)
{
    void *ptr=mmap(NULL,size,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_NORESERVE,-1,0);
    if(ptr==MAP_FAILED) FATAL_ERROR("Failed to allocate "+Value_To_String(size)+" bytes");
    if(0xfffUL&(uint64_t)ptr) FATAL_ERROR("Allocated pointer value "+Value_To_String(ptr)+" is not page-aligned");
    return ptr;
}

//#####################################################################
// Function Raw_Deallocate
//#####################################################################
void Raw_Deallocate(void* data, const size_t size)
{
    if(munmap(data,size)!=0) FATAL_ERROR("Failed to deallocate "+Value_To_String(size)+" bytes");
}
//#####################################################################
// Function Deactivate_Page
//#####################################################################
void Deactivate_Page(void* data, const size_t size)
{
    printf("Deactive_Page in SPGrid is no longer supported");
    exit(-1);
    /*
    if(0xfffUL&(uint64_t)data) FATAL_ERROR("Allocated pointer value "+Value_To_String(data)+" is not page-aligned");
    int ret=madvise(data,size,MADV_DONTNEED);
    if(ret<0){char buffer[256];
        char *error_message=strerror_r(errno,buffer,256);
        FATAL_ERROR("Failed to deallocate "+Value_To_String(size)+" bytes, ERROR: "+error_message);}
        */
}
//#####################################################################
// Function Check_Address_Resident
//#####################################################################
void Check_Address_Resident(const void* addr)
{
    printf("Not implemented\n");
    exit(-1);
}
//#####################################################################
// Function Validate_Memory_Use
//#####################################################################
// WARNING: This validation function is very conservative. It can fail (without truly indicating a problem) when using transparent hugepages
void Validate_Memory_Use(uint64_t number_of_pages,void *data_ptr,uint64_t *page_mask_array)
{
  printf("Not implemented\n");
  exit(-1);
}
//#####################################################################
}

