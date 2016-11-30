// GetFullPath.c
// Get absolute canonical path of a file or folder [WINDOWS]
// Absolute path names are safer than relative paths, when e.g. a GUI or TIMER
// callback changes the current directory. Only canonical paths without "." and
// ".." can be recognized uniquely.
// Long path names (>259 characters) require a magic initial key "\\?\" to be
// handled by Windows API functions, e.g. for Matlab's FOPEN, DIR and EXIST.
//
// FullName = GetFullPath(Name, Style)
// INPUT:
//   Name:  String or cell string, absolute or relative name of a file or
//          folder. The path need not exist. Unicode strings, UNC paths and long
//          names are supported.
//   Style: Style of the output as string, optional, default: 'auto'.
//          'auto': Add '\\?\' or '\\?\UNC\' for long names on demand.
//          'lean': Magic string is not added.
//          'fat':  Magic string is added for short names also.
//          The Style is ignored when not running under Windows.
//
// OUTPUT:
//   FullName: Absolute canonical path name as string or cell string.
//          For empty strings the current directory is replied.
//          '\\?\' or '\\?\UNC' is added on demand.
//
// NOTE: The M- and the MEX-version create the same results, the faster MEX
//   function works under Windows only.
//   Some functions of the Windows-API still do not support long file names.
//   E.g. the Recycler and the Windows Explorer fail even with the magic '\\?\'
//   prefix. Some functions of Matlab accept 260 characters (value of MAX_PATH),
//   some at 259 already. Don't blame me.
//   The 'fat' style is useful e.g. when Matlab's DIR command is called for a
//   folder with les than 260 characters, but together with the file name this
//   limit is exceeded. Then "dir(GetFullPath([folder, '\*.*], 'fat'))" helps.
//
// EXAMPLES:
//   cd(tempdir);                    % Assumed as 'C:\Temp' here
//   GetFullPath('File.Ext')         % 'C:\Temp\File.Ext'
//   GetFullPath('..\File.Ext')      % 'C:\File.Ext'
//   GetFullPath('..\..\File.Ext')   % 'C:\File.Ext'
//   GetFullPath('.\File.Ext')       % 'C:\Temp\File.Ext'
//   GetFullPath('*.txt')            % 'C:\Temp\*.txt'
//   GetFullPath('..')               % 'C:\'
//   GetFullPath('Folder\')          % 'C:\Temp\Folder\'
//   GetFullPath('D:\A\..\B')        % 'D:\B'
//   GetFullPath('\\Server\Folder\Sub\..\File.ext')
//                                   % '\\Server\Folder\File.ext'
//   GetFullPath({'..', 'new'})      % {'C:\', 'C:\Temp\new'}
//   GetFullPath('.', 'fat')         % '\\?\C:\Temp\File.Ext'
//
// COMPILE:
//   Beginners: InstallMex GetFullPath.c uTest_GetFullPath
//   Advanced:  mex -O GetFullPath.c
//   Download:  http://www.n-simon.de/mex
//   Run the unit-test uTest_GetFullPath after compiling.
//
// Tested: Matlab 6.5, 7.7, 7.8, 7.13, WinXP/32, Win7/64
//         Compiler: LCC2.4/3.8, BCC5.5, OWC1.8, MSVC2008/2010
// Assumed Compatibility: higher Matlab versions
// Author: Jan Simon, Heidelberg, (C) 2009-2013 matlab.THISYEAR(a)nMINUSsimon.de
//
// See also: CD, FULLFILE, FILEPARTS.

/*
% $JRev: R-E V:030 Sum:/wpAgQzWRWH1 Date:13-Jan-2013 18:29:11 $
% $License: BSD (use/copy/change/redistribute on own risk, mention the author) $
% $UnitTest: uTest_GetFullPath $
% $File: Tools\Mex\Source\GetFullPath.c $
% History:
% 001: 19-Apr-2010 01:23, Successor of Rel2AbsPath.
%      No check of validity or existence in opposite to Rel2AbsPath.
% 011: 24-Jan-2011 11:38, Cell string as input.
% 013: 27-Apr-2011 10:29, Minor bug: Bad ID for error messages.
% 019: 09-Aug-2012 14:00, In MEX: A leading "//" need not be a UNC path.
%      The former version treated "\\?\C:\<longpath>\file" as UNC path and
%      replied "\\?\UNC\?\C:\<longpath>\file".
% 028: 01-Jan-2013 14:35, Auto, Lean and Fat output style.
*/

#if !defined(__WINDOWS__) && !defined(_WIN32) && !defined(_WIN64)
#  error Sorry: Implemented for Windows only now!
#endif

#include <windows.h>
#include <mex.h>
#include <string.h>
#include <wchar.h>

// Assume 32 bit addressing for Matlab 6.5:
// See MEX option "compatibleArrayDims" for MEX in Matlab >= 7.7.
#ifndef MWSIZE_MAX
#define mwSize  int32_T           // Defined in tmwtypes.h
#define mwIndex int32_T
#define MWSIZE_MAX MAX_int32_T
#endif

// Error messages do not contain the function name in Matlab 6.5! This is not
// necessary in Matlab 7, but it does not bother:
#define ERR_HEAD "*** GetFullPath: "
#define ERR_ID   "JSimon:GetFullPath:"

// Static memory for strings:
DWORD   StaticBufferLen = MAX_PATH + 1;
wchar_t StaticBuffer_W[MAX_PATH + 1],    // Static buffer
        *Prefix = L"\\\\?\\UNC\\";       // Magic string for long names

// Do not accept ridiculously long file names:
#define MAXFILELEN_INPUT 8191L   // Arbitrary, 32767 is fine also

// Type of the output:
enum OutStyle_ENUM {Auto_Style, Lean_Style, Fat_Style};
typedef enum OutStyle_ENUM OutStyle_T;

// Prototypes:
mxArray *Core(mxChar *Name_M, mwSize NameLenL, OutStyle_T OutStyle);

// Main function ===============================================================
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  const mxArray *In, *String;
  mxArray *Out;
  mwSize  iC;
  OutStyle_T OutStyle = Auto_Style;  // Default: Auto-expanding
  char    StyleIn;

  // Check number of inputs and outputs:
  if (nrhs == 0 || nrhs > 2 || nlhs > 1) {
     mexErrMsgIdAndTxt(ERR_ID   "BadNInput",
                       ERR_HEAD "2 inputs allowed, 1 output allowed.");
  }

  // Parse 2nd input, if it is not empty:
  if (nrhs == 2 && !mxIsEmpty(prhs[1])) {
     if (!mxIsChar(prhs[1])) {
        mexErrMsgIdAndTxt(ERR_ID   "BadType",
                          ERR_HEAD "2nd input [Style] must be a string.");
     }

     // "Auto", "Lean", "Fat":
     StyleIn = *((char *) mxGetData(prhs[1]));
     switch (StyleIn) {
        case 'a':  // Fallthrough
        case 'A':  OutStyle = Auto_Style;  break;
        case 'l':  // Fallthrough
        case 'L':  OutStyle = Lean_Style;  break;
        case 'f':  // Fallthrough
        case 'F':  OutStyle = Fat_Style;   break;
        default:
           mexErrMsgIdAndTxt(ERR_ID   "BadStyle",
                             ERR_HEAD "Unknown Style.");
     }
  }

  // Get input string or cell string and call the core: ------------------------
  In = prhs[0];
  if (mxIsChar(In)) {                          // Input is a string:
     plhs[0] = Core((mxChar *) mxGetData(In), mxGetNumberOfElements(prhs[0]),
                    OutStyle);

  } else if (mxIsCell(In)) {                   // Input is a cell string:
     plhs[0] = mxCreateCellArray(mxGetNumberOfDimensions(In),
                                 mxGetDimensions(In));
     Out = plhs[0];
     iC  = mxGetNumberOfElements(In);
     while (iC-- > 0) {                        // Backwards, faster than FOR
        String = mxGetCell(In, iC);
        if (String == NULL) {                  // Uninitialized cell:
           mxSetCell(Out, iC,
                     Core(NULL, 0, OutStyle)); // Reply current directory
        } else if (mxIsChar(String)) {         // Cell element is a string
           mxSetCell(Out, iC,
                     Core((mxChar *) mxGetData(String),
                          mxGetNumberOfElements(String), OutStyle));
        } else {                               // Bad cell element
           mexErrMsgIdAndTxt(ERR_ID "BadInputType",
                        ERR_HEAD "[FileName] must be a string or cell string.");
        }
     }

  } else {                                     // Bad input type:
     mexErrMsgIdAndTxt(ERR_ID   "BadInputType",
                       ERR_HEAD "[FileName] must be a string or cell string.");
  }

  return;
}

// =============================================================================
mxArray *Core(mxChar *Name_M, mwSize NameLenL, OutStyle_T OutStyle)
{
  // Convert input string to full qualified and canonical path.
  // INPUT:
  //   Name_M:  Pointer to string in Matlab format, no terminator
  //   NameLen: Number of characters without terminator
  // OUTPUT:
  //   mxArray pointer: Matlab array containing the full path as string.

  wchar_t *Name_W, *Full_W, *Out_W, *Buffer_W;  // "w"ide character
  bool    freeName, freeBuffer, isUNC;
  mwSize  dims[2] = {1L, 0L};
  mxArray *Full_M;                              // "M"atlab variable
  DWORD   FullLen, In_Offset, Out_Shift, Out_Crop, BufferLen,
          NameLen = (DWORD) NameLenL;

  // Convert input string to terminated 2-byte unicode string: -----------------
  if (NameLen == 0) {          // Empty input => Current directory:
     Name_W   = L".";          // Terminated, is converted to current folder
     freeName = false;

  } else if (NameLen <= MAXFILELEN_INPUT) {
     // Copy Matlab string to terminated 2-Byte Unicode string:
     freeName = true;
     Name_W   = (wchar_t *) mxMalloc((NameLen + 1) * sizeof(wchar_t));
     if (Name_W == NULL) {
        mexErrMsgIdAndTxt(ERR_ID   "NoMemory",
                          ERR_HEAD "No memory for FileName.");
     }
     memcpy(Name_W, Name_M, NameLen * sizeof(wchar_t));
     Name_W[NameLen] = L'\0';

  } else {                     // Ridiculous input: ----------------------------
     mexErrMsgIdAndTxt(ERR_ID   "BadInputSize",
                       ERR_HEAD "FileName is too long: %d characters",
                       MAXFILELEN_INPUT);
  }

  // Input cases:
  //   short                 -> no input offset
  //   \\server\share\short  -> no input offset
  //   \\server\share\long   -> no input offset
  //   long                  -> no input offset
  //   \\?\any               -> input offset = 4
  //   \\?\UNC\server\share  -> input offset = 6, insert leading '\'

  // Ignore leading "\\?\" and "\\?\UNC\". Otherwise the Windows API function
  // GetFullPathName("\\?\D:\..") replies "\\?\D:" instead of "\\?\D:\" !?
  In_Offset = 0;
  isUNC     = false;
  if (wcsncmp(Name_W, Prefix, 2) == 0) {                 // \\...
     if (wcsncmp(Name_W + 2, Prefix + 2, 2) == 0) {      // \\?\...
        if (wcsncmp(Name_W + 4, Prefix + 4, 4) == 0) {   // \\?\UNC\path
           // "\\?\UNC\server\share" -> "\\server\share"
           isUNC     = true;
           In_Offset = 6;
           Name_W[6] = L'\\';                            // \\server\share
        } else {                                         // \\?\path
           // "\\?\UNC\path" -> "path"
           In_Offset = 4;
        }
     } else {                                            // \\server
        isUNC = true;
     }
  }

  // Define Buffer for output: -------------------------------------------------
  if (NameLen - In_Offset < StaticBufferLen) {    // short input name
     freeBuffer = false;
     BufferLen  = StaticBufferLen;
     Buffer_W   = StaticBuffer_W;
  } else {                                        // long input name
     // Most likely the output has a shorter or equal length as the input:
     freeBuffer = true;
     BufferLen  = NameLen + 8;                    // "\\?\UNC" plus terminator
     Buffer_W   = (wchar_t *) mxMalloc(BufferLen * sizeof(wchar_t));
     if (Buffer_W == NULL) {
        mexErrMsgIdAndTxt(ERR_ID   "NoMemory",
                          ERR_HEAD "No memory for Buffer.");
     }
  }

  // Call Windows API to get the full path: ------------------------------------
  FullLen = GetFullPathNameW(
                      (LPCWSTR) Name_W + In_Offset,   // file name
                      BufferLen,          // length of static buffer
                      (LPWSTR) Buffer_W,  // static buffer [MAX_PATH]
                      NULL);              // unused, pointer to filename in path

  if (FullLen == 0) {                     // GetFullPathName failed: -----------
     // Not expected!
     mexErrMsgIdAndTxt(ERR_ID   "API_failed_static",
                       ERR_HEAD "WindowsAPI:GetFullPathName failed.");
  }

  // Check if the full name is still too long, although the buffer is larger
  // than the input already:
  if (FullLen > BufferLen) {              // Full name too long: ---------------
     // SECURITY FALLBACK ONLY! This is an unexpected exception, because the
     // buufer length has been determined appropriately already.
     if (freeBuffer) {                    // Free formerly created buffer:
        mxFree(Buffer_W);
     }

     // Create bigger buffer, FullLen plus terminator:
     freeBuffer = true;
     BufferLen  = FullLen + 1;
     Buffer_W   = (wchar_t *) mxCalloc(BufferLen, sizeof(wchar_t));
     if (Buffer_W == NULL) {
        mexErrMsgIdAndTxt(ERR_ID   "NoMemory",
                          ERR_HEAD "No memory for dynamic buffer.");
     }

     // Call Windows API again to get the full path:
     FullLen = GetFullPathNameW(
                      (LPCWSTR) Name_W + In_Offset,   // file name
                      BufferLen,          // buffer length, with terminator
                      Buffer_W,           // address of path buffer
                      NULL);              // address of filename in path

     if (FullLen == 0) {                  // GetFullPathNameW failed:
        mexErrMsgIdAndTxt(ERR_ID   "API_failed_dynamic",
                          ERR_HEAD "Win:GetFullPathName failed (long name).");
     }
  }

  // Add the magic prefix //?/ on demand:
  if ((OutStyle == Auto_Style && FullLen >= MAX_PATH) ||
       OutStyle == Fat_Style) {
     // Either automatic expanding is enabled and the output length exceeds
     // MAX_PATH, or fat style:
     //   short                -> no output offset
     //   \\server\share\short -> no output offset
     //   long                 -> output offset=4, add \\?\
     //   \\server\share\long  -> output offset=6, add \\?\UNC, delete leading \
     // Not possible, because GetFullPathName does not insert the prefix:
     //   # \\?\short
     //   # \\?\UNC\server\share\short
     //   # \\?\long
     //   # \\?\UNC\server\share\long

     // Check for UNC path again, because the current value of isUNC can be
     // FALSE, when the input is a relative file name and the current folder is
     // UNC path. Here a leading '\\' cannot be a '\\?\'.
     if (wcsncmp(Buffer_W, Prefix, 2)) {  // No UNC path:
        Out_Shift = 4;                  // Shift output to right
        Out_Crop  = 0;
     } else {                           // UNC Path:
        Out_Shift = 7;                  // Shift output to right, insert prefix
        Out_Crop  = 1;                  // Remove leading characters
     }

  } else {  // No shifting in lean style or for short name:
     Out_Crop  = 0;
     Out_Shift = 0;
  }

  // Create output as Matlab string:
  FullLen -= Out_Crop;
  dims[1]  = FullLen + Out_Shift;
  Full_M   = mxCreateCharArray(2, dims);
  Out_W    = (wchar_t *) mxGetData(Full_M);

  // Copy strings to output:
  if (Out_Shift == 0) {
     memcpy(Out_W, Buffer_W, FullLen * sizeof(mxChar));
  } else {
     // Insert prefix "\\?\" or "\\?\UNC" at first:
     memcpy(Out_W, Prefix, Out_Shift * sizeof(mxChar));
     memcpy(Out_W + Out_Shift, Buffer_W + Out_Crop, FullLen * sizeof(mxChar));
  }

  // Cleanup:
  if (freeName) {
     mxFree(Name_W);
  }
  if (freeBuffer) {
     mxFree(Buffer_W);
  }

  return Full_M;
}
