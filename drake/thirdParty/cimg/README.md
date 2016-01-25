<a href="http://cimg.eu">![Logo](http://cimg.eu/img/CImgLogo2.jpg)</a>
##### http://cimg.eu

------------------

The **CImg Library** is a **small**, **open-source**, and **modern C++ toolkit** for **image processing**, designed with these properties in mind:

![Usefulness](http://cimg.eu/img/item_usefulness.jpg) **CImg** defines *classes* and *methods* to manage images in your own C++ code. You can use **CImg** to load/save various file formats, access pixel values, display/transform/filter images, draw primitives (text, faces, curves, 3d objects, ...), compute statistics, manage user interactions on images, and so on...

![Genericity](http://cimg.eu/img/item_genericity.jpg) **CImg** defines a single image class able to represent datasets having up to *4-dimensions* (from 1d scalar signals to 3d hyperspectral volumetric images), with *template pixel types* (`bool,char,int,float,...`). It also handles image *collections* and *sequences*.

![Portability](http://cimg.eu/img/item_portability.jpg) **CImg** is *self-contained*, *thread-safe* and *highly portable*. It fully works on *different operating systems* (`Unix,Windows,MacOS X,*BSD,...`) and is compatible with *various C++ compilers* (`Visual C++,g++,clang++,icc,...`).

![Simplicity](http://cimg.eu/img/item_simplicity.jpg) **CImg** is *lightweight*. It is made of a single header file [`CImg.h`](https://raw.githubusercontent.com/dtschump/CImg/master/CImg.h) that must be included in your C++ source. It defines only *four* different classes, encapsulated in the namespace `cimg_library`. It can be compiled using a minimal set of standard C++ and system libraries only. *No need for exotic or complex dependencies*.

![Extensibility](http://cimg.eu/img/item_extensibility.jpg) Although not mandatory, **CImg** can use functionalities of external tools/libraries such as [Board](http://libboard.sourceforge.net/), [FFMPEG](http://ffmpeg.mplayerhq.hu/), [FFTW3](http://www.fftw.org/), [GraphicsMagick](http://www.graphicsmagick.org/), [ImageMagick](http://www.imagemagick.org/), [Lapack](http://www.netlib.org/lapack/), [libcurl](http://curl.haxx.se/libcurl/), [libjpeg](http://www.ijg.org/), [libpng](http://www.libpng.org/pub/png/libpng.html), [libtiff](http://www.libtiff.org/), [Magick++](http://www.imagemagick.org/Magick++/), [OpenEXR](http://www.openexr.com/), [OpenCV](http://http://opencv.willowgarage.com/wiki/), [OpenMP](http://www.openmp.org/) or [XMedCon](http://xmedcon.sourceforge.net/). Moreover, a simple *plug-in* mechanism allows any user to directly enhance the library capabilities according to his needs.

![Freedom](http://cimg.eu/img/item_freedom.jpg) **CImg** is a *free, open-source library* distributed under the [*CeCILL-C*](http://www.cecill.info/licences/Licence_CeCILL-C_V1-en.txt) (close to the GNU LGPL) or [CeCILL](http://www.cecill.info/licences/Licence_CeCILL_V2-en.txt) (compatible with the GNU GPL) licenses. It can be used in commercial applications.

------------------

> **CImg** stands for **Cool Image** : It is _easy to use_, _efficient_ and is intended to be a very pleasant toolbox to design image processing algorithms in C++. Due to its generic conception, it can cover a wide range of image processing applications.

------------------