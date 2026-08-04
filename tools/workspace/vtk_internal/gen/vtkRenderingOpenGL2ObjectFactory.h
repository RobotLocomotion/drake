// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

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

  const char * GetDescription() VTK_FUTURE_CONST override { return "vtkRenderingOpenGL2 factory overrides."; }

  const char * GetVTKSourceVersion() VTK_FUTURE_CONST override;

  void PrintSelf(ostream &os, vtkIndent indent) override;

protected:
  vtkRenderingOpenGL2ObjectFactory();

private:
  vtkRenderingOpenGL2ObjectFactory(const vtkRenderingOpenGL2ObjectFactory&) = delete;
  void operator=(const vtkRenderingOpenGL2ObjectFactory&) = delete;
};

VTK_ABI_NAMESPACE_END

#endif // vtkRenderingOpenGL2ObjectFactory_h
