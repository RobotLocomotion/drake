// SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
// SPDX-License-Identifier: BSD-3-Clause

// Object factories should never have deprecation warnings.
// NOLINTNEXTLINE(modernize-macro-to-enum)
#define VTK_DEPRECATION_LEVEL 0

#include "vtkRenderingOpenGL2ObjectFactory.h"
#include "vtkVersion.h"

// Include all of the classes we want to create overrides for.
#include "vtkDummyGPUInfoList.h"
#include "vtkOpenGLActor.h"
#include "vtkOpenGLCompositePolyDataMapperDelegator.h"
#include "vtkOpenGLBillboardTextActor3D.h"
#include "vtkOpenGLCamera.h"
#include "vtkOpenGLCameraPass.h"
#include "vtkOpenGLClearRGBPass.h"
#include "vtkOpenGLLabeledContourMapper.h"
#include "vtkOpenGLHardwareSelector.h"
#include "vtkOpenGLImageMapper.h"
#include "vtkOpenGLImageSliceMapper.h"
#include "vtkOpenGLGlyph3DMapper.h"
#include "vtkOpenGLLight.h"
#include "vtkOpenGLPointGaussianMapper.h"
#include "vtkOpenGLPolyDataMapper.h"
#include "vtkOpenGLPolyDataMapper2D.h"
#include "vtkOpenGLProperty.h"
#include "vtkOpenGLShaderProperty.h"
#include "vtkOpenGLUniforms.h"
#include "vtkOpenGLRenderer.h"
#include "vtkOpenGLRenderTimerLog.h"
#include "vtkOpenGLSkybox.h"
#include "vtkOpenGLTextActor.h"
#include "vtkOpenGLTextActor3D.h"
#include "vtkOpenGLTextMapper.h"
#include "vtkOpenGLTexture.h"
// Removed the next lines for Drake.
// #include "vtkOpenGLBatchedLabeledDataMapper.h"
// #include "vtkOpenGLHyperTreeGridMapper.h"
// #include "vtkOpenGLRenderWindow.h"


VTK_ABI_NAMESPACE_BEGIN

vtkStandardNewMacro(vtkRenderingOpenGL2ObjectFactory);

// Now create the functions to create overrides with.
VTK_CREATE_CREATE_FUNCTION(vtkDummyGPUInfoList)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLActor)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLCompositePolyDataMapperDelegator)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLBillboardTextActor3D)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLCamera)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLCameraPass)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLClearRGBPass)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLLabeledContourMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLHardwareSelector)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLImageMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLImageSliceMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLGlyph3DMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLLight)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLPointGaussianMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLPolyDataMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLPolyDataMapper2D)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLProperty)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLShaderProperty)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLUniforms)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLRenderer)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLRenderTimerLog)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLSkybox)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLTextActor)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLTextActor3D)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLTextMapper)
VTK_CREATE_CREATE_FUNCTION(vtkOpenGLTexture)
// Removed the next lines for Drake.
// VTK_CREATE_CREATE_FUNCTION(vtkOpenGLBatchedLabeledDataMapper)
// VTK_CREATE_CREATE_FUNCTION(vtkOpenGLHyperTreeGridMapper)
// VTK_CREATE_CREATE_FUNCTION(vtkOpenGLRenderWindow)


vtkRenderingOpenGL2ObjectFactory::vtkRenderingOpenGL2ObjectFactory()
{
this->RegisterOverride("vtkGPUInfoList",
      "vtkDummyGPUInfoList",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkDummyGPUInfoList,
      #if defined(vtkDummyGPUInfoList_OVERRIDE_ATTRIBUTES)
      vtkDummyGPUInfoList_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkActor",
      "vtkOpenGLActor",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLActor,
      #if defined(vtkOpenGLActor_OVERRIDE_ATTRIBUTES)
      vtkOpenGLActor_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkCompositePolyDataMapperDelegator",
      "vtkOpenGLCompositePolyDataMapperDelegator",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLCompositePolyDataMapperDelegator,
      #if defined(vtkOpenGLCompositePolyDataMapperDelegator_OVERRIDE_ATTRIBUTES)
      vtkOpenGLCompositePolyDataMapperDelegator_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkBillboardTextActor3D",
      "vtkOpenGLBillboardTextActor3D",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLBillboardTextActor3D,
      #if defined(vtkOpenGLBillboardTextActor3D_OVERRIDE_ATTRIBUTES)
      vtkOpenGLBillboardTextActor3D_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkCamera",
      "vtkOpenGLCamera",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLCamera,
      #if defined(vtkOpenGLCamera_OVERRIDE_ATTRIBUTES)
      vtkOpenGLCamera_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkCameraPass",
      "vtkOpenGLCameraPass",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLCameraPass,
      #if defined(vtkOpenGLCameraPass_OVERRIDE_ATTRIBUTES)
      vtkOpenGLCameraPass_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkClearRGBPass",
      "vtkOpenGLClearRGBPass",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLClearRGBPass,
      #if defined(vtkOpenGLClearRGBPass_OVERRIDE_ATTRIBUTES)
      vtkOpenGLClearRGBPass_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkLabeledContourMapper",
      "vtkOpenGLLabeledContourMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLLabeledContourMapper,
      #if defined(vtkOpenGLLabeledContourMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLLabeledContourMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkHardwareSelector",
      "vtkOpenGLHardwareSelector",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLHardwareSelector,
      #if defined(vtkOpenGLHardwareSelector_OVERRIDE_ATTRIBUTES)
      vtkOpenGLHardwareSelector_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkImageMapper",
      "vtkOpenGLImageMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLImageMapper,
      #if defined(vtkOpenGLImageMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLImageMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkImageSliceMapper",
      "vtkOpenGLImageSliceMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLImageSliceMapper,
      #if defined(vtkOpenGLImageSliceMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLImageSliceMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkGlyph3DMapper",
      "vtkOpenGLGlyph3DMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLGlyph3DMapper,
      #if defined(vtkOpenGLGlyph3DMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLGlyph3DMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkLight",
      "vtkOpenGLLight",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLLight,
      #if defined(vtkOpenGLLight_OVERRIDE_ATTRIBUTES)
      vtkOpenGLLight_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkPointGaussianMapper",
      "vtkOpenGLPointGaussianMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLPointGaussianMapper,
      #if defined(vtkOpenGLPointGaussianMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLPointGaussianMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkPolyDataMapper",
      "vtkOpenGLPolyDataMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLPolyDataMapper,
      #if defined(vtkOpenGLPolyDataMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLPolyDataMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkPolyDataMapper2D",
      "vtkOpenGLPolyDataMapper2D",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLPolyDataMapper2D,
      #if defined(vtkOpenGLPolyDataMapper2D_OVERRIDE_ATTRIBUTES)
      vtkOpenGLPolyDataMapper2D_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkProperty",
      "vtkOpenGLProperty",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLProperty,
      #if defined(vtkOpenGLProperty_OVERRIDE_ATTRIBUTES)
      vtkOpenGLProperty_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkShaderProperty",
      "vtkOpenGLShaderProperty",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLShaderProperty,
      #if defined(vtkOpenGLShaderProperty_OVERRIDE_ATTRIBUTES)
      vtkOpenGLShaderProperty_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkUniforms",
      "vtkOpenGLUniforms",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLUniforms,
      #if defined(vtkOpenGLUniforms_OVERRIDE_ATTRIBUTES)
      vtkOpenGLUniforms_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkRenderer",
      "vtkOpenGLRenderer",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLRenderer,
      #if defined(vtkOpenGLRenderer_OVERRIDE_ATTRIBUTES)
      vtkOpenGLRenderer_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkRenderTimerLog",
      "vtkOpenGLRenderTimerLog",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLRenderTimerLog,
      #if defined(vtkOpenGLRenderTimerLog_OVERRIDE_ATTRIBUTES)
      vtkOpenGLRenderTimerLog_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkSkybox",
      "vtkOpenGLSkybox",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLSkybox,
      #if defined(vtkOpenGLSkybox_OVERRIDE_ATTRIBUTES)
      vtkOpenGLSkybox_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkTextActor",
      "vtkOpenGLTextActor",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLTextActor,
      #if defined(vtkOpenGLTextActor_OVERRIDE_ATTRIBUTES)
      vtkOpenGLTextActor_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkTextActor3D",
      "vtkOpenGLTextActor3D",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLTextActor3D,
      #if defined(vtkOpenGLTextActor3D_OVERRIDE_ATTRIBUTES)
      vtkOpenGLTextActor3D_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkTextMapper",
      "vtkOpenGLTextMapper",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLTextMapper,
      #if defined(vtkOpenGLTextMapper_OVERRIDE_ATTRIBUTES)
      vtkOpenGLTextMapper_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
this->RegisterOverride("vtkTexture",
      "vtkOpenGLTexture",
      "Override for VTK::RenderingOpenGL2 module",
      1,
      vtkObjectFactoryCreatevtkOpenGLTexture,
      #if defined(vtkOpenGLTexture_OVERRIDE_ATTRIBUTES)
      vtkOpenGLTexture_OVERRIDE_ATTRIBUTES
      #else
      nullptr
      #endif
      );
// Removed the next lines for Drake.
// this->RegisterOverride("vtkBatchedLabeledDataMapper",
//       "vtkOpenGLBatchedLabeledDataMapper",
//       "Override for VTK::RenderingOpenGL2 module",
//       1,
//       vtkObjectFactoryCreatevtkOpenGLBatchedLabeledDataMapper,
//       #if defined(vtkOpenGLBatchedLabeledDataMapper_OVERRIDE_ATTRIBUTES)
//       vtkOpenGLBatchedLabeledDataMapper_OVERRIDE_ATTRIBUTES
//       #else
//       nullptr
//       #endif
//       );
// this->RegisterOverride("vtkHyperTreeGridMapper",
//       "vtkOpenGLHyperTreeGridMapper",
//       "Override for VTK::RenderingOpenGL2 module",
//       1,
//       vtkObjectFactoryCreatevtkOpenGLHyperTreeGridMapper,
//       #if defined(vtkOpenGLHyperTreeGridMapper_OVERRIDE_ATTRIBUTES)
//       vtkOpenGLHyperTreeGridMapper_OVERRIDE_ATTRIBUTES
//       #else
//       nullptr
//       #endif
//       );
// this->RegisterOverride("vtkRenderWindow",
//       "vtkOpenGLRenderWindow",
//       "Override for VTK::RenderingOpenGL2 module",
//       1,
//       vtkObjectFactoryCreatevtkOpenGLRenderWindow,
//       #if defined(vtkOpenGLRenderWindow_OVERRIDE_ATTRIBUTES)
//       vtkOpenGLRenderWindow_OVERRIDE_ATTRIBUTES
//       #else
//       nullptr
//       #endif
//       );

}

const char * vtkRenderingOpenGL2ObjectFactory::GetVTKSourceVersion() VTK_FUTURE_CONST
{
  return VTK_SOURCE_VERSION;
}

void vtkRenderingOpenGL2ObjectFactory::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

// Registration of object factories.
static unsigned int vtkRenderingOpenGL2Count = 0;

VTKRENDERINGOPENGL2_EXPORT void vtkRenderingOpenGL2_AutoInit_Construct()
{
  if(++vtkRenderingOpenGL2Count == 1)
  {


    vtkRenderingOpenGL2ObjectFactory* factory = vtkRenderingOpenGL2ObjectFactory::New();
    if (factory)
    {
      // vtkObjectFactory keeps a reference to the "factory",
      vtkObjectFactory::RegisterFactory(factory);
      factory->Delete();
    }
  }
}
VTK_ABI_NAMESPACE_END
