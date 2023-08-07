/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkRenderingOpenGL2ObjectFactory.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#ifndef vtkRenderingOpenGL2ObjectFactory_h
#define vtkRenderingOpenGL2ObjectFactory_h

#include "vtkRenderingOpenGL2Module.h" // For export macro
#include "vtkObjectFactory.h"

VTK_ABI_NAMESPACE_BEGIN

class VTKRENDERINGOPENGL2_EXPORT vtkRenderingOpenGL2ObjectFactory : public vtkObjectFactory
{
public:
  static vtkRenderingOpenGL2ObjectFactory * New();
  vtkTypeMacro(vtkRenderingOpenGL2ObjectFactory, vtkObjectFactory);

  const char * GetDescription() override { return "vtkRenderingOpenGL2 factory overrides."; }

  const char * GetVTKSourceVersion() override;

  void PrintSelf(ostream &os, vtkIndent indent) override;

protected:
  vtkRenderingOpenGL2ObjectFactory();

private:
  vtkRenderingOpenGL2ObjectFactory(const vtkRenderingOpenGL2ObjectFactory&) = delete;
  void operator=(const vtkRenderingOpenGL2ObjectFactory&) = delete;
};

VTK_ABI_NAMESPACE_END

#endif // vtkRenderingOpenGL2ObjectFactory_h
