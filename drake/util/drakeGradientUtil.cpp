#include "drakeGradientUtil.h"




// explicit instantiations
#define MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols) \
		template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Matrix<Type, BRows, BCols>>::type\
		matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >& dA, const Eigen::MatrixBase< Eigen::Matrix<Type, BRows, BCols> >& b);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, 1, 3, 3)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 6, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 4, 4)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, Eigen::Dynamic)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 36, Eigen::Dynamic, 6, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 1)
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, 6, 3, 3);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 9, 3, 3, 1);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 12, 3, 3, 1);
MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION(double, 12, 4, 4, 1);
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Map< Eigen::Matrix<Type, BRows, 1> > >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Map< Eigen::Matrix<Type, BRows, 1> > >&);
MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_MATGRADMULT_MAP_B_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols, BBlockRows, BBlockCols, InnerPanel) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Block< Eigen::Matrix<Type, BRows, BCols> const, BBlockRows, BBlockCols, InnerPanel> >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix<Type, BRows, BCols> const, BBlockRows, BBlockCols, InnerPanel> >&);
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 1, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 4, 4, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1, true)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 3, 1, 3, 1, true)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 6, 3, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, Eigen::Dynamic, 1, Eigen::Dynamic, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 6, Eigen::Dynamic, 6, 1, true)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 1, Eigen::Dynamic, 1, false)
MAKE_MATGRADMULT_BLOCK_B_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 6, 1, 3, 1, false)
#undef MAKE_MATGRADMULT_EXPLICIT_INSTANTIATION

template DLLEXPORT MatGradMult<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Block<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0, Eigen::Stride<0, 0> > const, -1, 1, false> >::type matGradMult(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1>, 0, Eigen::Stride<0, 0> > const, -1, 1, false> > const&);
template DLLEXPORT MatGradMult<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, 1, true> const, -1, 1, false> >::type matGradMult(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Block<Eigen::Matrix<double, -1, -1, 0, -1, -1>, -1, 1, true> const, -1, 1, false> > const&);
template DLLEXPORT MatGradMult<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Block<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::Stride<0, 0> > const, -1, 1, false> >::type matGradMult<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Block<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::Stride<0, 0> > const, -1, 1, false> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Block<Eigen::Map<Eigen::Matrix<double, -1, 1, 0, -1, 1> const, 0, Eigen::Stride<0, 0> > const, -1, 1, false> > const&);


#define MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(Type, DARows, DACols, BRows, BCols, BBlockRows, BBlockCols, InnerPanel) \
    template DLLEXPORT MatGradMult<Eigen::Matrix<Type, DARows, DACols>, Eigen::Block< Eigen::Transpose< Eigen::Matrix<Type, BRows, BCols> > const, BBlockRows, BBlockCols, InnerPanel> >::type \
    matGradMult(const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, DACols> >&, const Eigen::MatrixBase< Eigen::Block< Eigen::Transpose< Eigen::Matrix<Type, BRows, BCols> > const, BBlockRows, BBlockCols, InnerPanel> >&);
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, Eigen::Dynamic, 3, 1, false)
MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic, 6, 6, 3, 1, false)
#undef MAKE_MATGRADMULT_TRANSPOSE_BLOCK_B_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
		template DLLEXPORT MatGradMultMat<Eigen::Matrix<Type, ARows, ACols>, Eigen::Matrix<Type, ACols, BCols>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
		    const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 1, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 8, 6, 9, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 3, 4, 4, 12, 16, 4)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 4, 3, 3, 12, 9, 4)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 4, 4, 4, 16, 16, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 3, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 6, 6, Eigen::Dynamic, 36, Eigen::Dynamic, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double, 6, 6, 1, 36, 6, Eigen::Dynamic)
MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION(double,3,3,3,9,9,6)
#undef MAKE_MATGRADMULTMAT_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template DLLEXPORT MatGradMultMat<Eigen::Transpose<Eigen::Matrix<Type, ACols, ARows>>, Eigen::Matrix<Type, ACols, BCols>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, ACols, ARows>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ACols, BCols> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(double, 3, 3, 4, 9, 12, 4)
MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 6, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_A_EXPLICIT_INSTANTIATION

#define MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(Type, ARows, ACols, BCols, DARows, DBRows, NQ) \
    template DLLEXPORT MatGradMultMat<Eigen::Matrix<Type, ARows, ACols>, Eigen::Transpose<Eigen::Matrix<Type, BCols, ACols>>, Eigen::Matrix<Type, DARows, NQ>>::type matGradMultMat(\
        const Eigen::MatrixBase< Eigen::Matrix<Type, ARows, ACols> >&,\
        const Eigen::MatrixBase< Eigen::Transpose<Eigen::Matrix<Type, BCols, ACols>> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DARows, NQ> >&,\
        const Eigen::MatrixBase< Eigen::Matrix<Type, DBRows, NQ> >&);
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 3, 1, 3, 3, 3, 3)
MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION(double, 4, 1, 4, 4, 4, 4)
#undef MAKE_MATGRADMULTMAT_TRANSPOSE_B_EXPLICIT_INSTANTIATION

template DLLEXPORT MatGradMultMat<Eigen::Matrix<double, -1, 3, 0, -1, 3>, Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 3, -1, false>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >::type matGradMultMat<Eigen::Matrix<double, -1, 3, 0, -1, 3>,
    Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 3, -1, false>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, 3, 0, -1, 3> > const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<double, 6, -1, 0, 6, -1>, 3, -1, false> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&);

#define MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, NRows, NCols) \
		template DLLEXPORT void setSubMatrixGradient<QSubvectorSize,Eigen::Matrix<Type, DMRows, DMCols>, Eigen::Matrix<Type, DMSubRows, DMSubCols>, NRows, NCols>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const Eigen::MatrixBase< Eigen::Matrix<Type, DMSubRows, DMSubCols> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index, Eigen::Matrix<Type, DMSubRows, DMSubCols>::Index);
//MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 6, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 9, Eigen::Dynamic, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 16, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 3, 4)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 12, 4, 4, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 3, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, 4, Eigen::Dynamic, Eigen::Dynamic, 9, 4, 4, 3)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 36, Eigen::Dynamic, 3, Eigen::Dynamic, 3, 1)
MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 6, Eigen::Dynamic, 6, 1)
#undef MAKE_SETSUBMATRIXGRADIENT_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, DMSubRows, DMSubCols) \
  template DLLEXPORT void setSubMatrixGradient(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const Eigen::MatrixBase< Eigen::Matrix<Type, DMSubRows, DMSubCols> >&, const std::vector<int>&, const std::vector<int>&, Eigen::Matrix<Type, DMRows, DMCols>::Index, Eigen::Matrix<Type, DMRows, DMCols>::Index, Eigen::Matrix<Type, DMRows, DMCols>::Index);
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 9, Eigen::Dynamic)
#undef MAKE_SETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubCols) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Matrix< Type, 1, DMSubCols > >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3)
#undef MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, DMSubBlockCols, InnerPanel) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix< Type, DMSubRows, DMSubCols> const, 1, DMSubBlockCols, InnerPanel> >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, false)
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, 3, Eigen::Dynamic, Eigen::Dynamic, false)
#undef MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_CONST_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols, DMSubRows, DMSubCols, DMSubBlockCols, InnerPanel) \
  template DLLEXPORT void setSubMatrixGradient<QSubvectorSize>(Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const Eigen::MatrixBase< Eigen::Block< Eigen::Matrix< Type, DMSubRows, DMSubCols>, 1, DMSubBlockCols, InnerPanel> >& dM_submatrix, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic, false)
#undef MAKE_SETSUBMATRIXGRADIENT_SINGLE_ELEMENT_BLOCK_SUBMATRIX_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, NRows, NCols, QSubvectorSize) \
	template DLLEXPORT GetSubMatrixGradientArray<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols>, NRows, NCols>::type getSubMatrixGradient<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols>, NRows, NCols>(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, Eigen::Matrix<Type, DMRows, DMCols>::Index, int, Eigen::Matrix<Type, DMRows, DMCols>::Index);
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 3, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 16, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 3, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, 36, Eigen::Dynamic, 3, 1, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, 3, 3, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_ARRAY_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_ARRAY_BLOCK_DM_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols, DMBlockRows, DMBlockCols, InnerPanel, NRows, NCols, QSubvectorSize) \
  template DLLEXPORT GetSubMatrixGradientArray<QSubvectorSize, Eigen::Block<Eigen::Matrix<Type, DMRows, DMCols>, DMBlockRows, DMBlockCols, InnerPanel>, NRows, NCols>::type getSubMatrixGradient<QSubvectorSize, Eigen::Block<Eigen::Matrix<Type, DMRows, DMCols>, DMBlockRows, DMBlockCols, InnerPanel>, NRows, NCols>(const Eigen::MatrixBase< Eigen::Block<Eigen::Matrix<Type, DMRows, DMCols>, DMBlockRows, DMBlockCols, InnerPanel> >&, const std::array<int, NRows>&, const std::array<int, NCols>&, Eigen::Block<Eigen::Matrix<Type, DMRows, DMCols>, DMBlockRows, DMBlockCols, InnerPanel>::Index, int, Eigen::Block<Eigen::Matrix<Type, DMRows, DMCols>, DMBlockRows, DMBlockCols, InnerPanel>::Index);
MAKE_GETSUBMATRIXGRADIENT_ARRAY_BLOCK_DM_EXPLICIT_INSTANTIATION(double, 6, Eigen::Dynamic, 6, Eigen::Dynamic, true, 3, 1, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_ARRAY_BLOCK_DM_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(Type, DMRows, DMCols) \
  template DLLEXPORT Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> getSubMatrixGradient(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, const std::vector<int>& rows, const std::vector<int>& cols, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, int q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_GETSUBMATRIXGRADIENT_VECTOR_EXPLICIT_INSTANTIATION

#define MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(Type, QSubvectorSize, DMRows, DMCols) \
  template DLLEXPORT GetSubMatrixGradientSingleElement<QSubvectorSize, Eigen::Matrix<Type, DMRows, DMCols> >::type \
getSubMatrixGradient<QSubvectorSize>(const Eigen::MatrixBase< Eigen::Matrix<Type, DMRows, DMCols> >& dM, int row, int col, Eigen::Matrix<Type, DMRows, DMCols>::Index M_rows, \
    Eigen::Matrix<Type, DMRows, DMCols>::Index q_start, Eigen::Matrix<Type, DMRows, DMCols>::Index q_subvector_size);
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 9, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, 3, Eigen::Dynamic)
MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION(double, 6, 9, 6)
#undef MAKE_GETSUBMATRIXGRADIENT_SINGLE_ELEMENT_EXPLICIT_INSTANTIATION

#define MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(Type, Rows, Cols) \
        template DLLEXPORT Eigen::Matrix<Type, Rows, Cols>::PlainObject transposeGrad(const Eigen::MatrixBase<Eigen::Matrix<Type, Rows, Cols>>&, Eigen::Matrix<Type, Rows, Cols>::Index);
//MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 3, 3)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 4, 4)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 48, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 9, 4)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, 36, Eigen::Dynamic)
MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION(double, Eigen::Dynamic, Eigen::Dynamic)
#undef MAKE_TRANSPOSEGRAD_EXPLICIT_INSTANTIATION
