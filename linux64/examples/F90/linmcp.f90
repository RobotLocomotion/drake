MODULE probData
  IMPLICIT NONE
  INTEGER, PARAMETER :: probDim=3
  REAL(8), DIMENSION(probDim,probDim) :: mat = reshape((/ 4, 1, -1, 1, 4, 1, -1, 1, 4 /),shape(mat))
  REAL(8), DIMENSION(probDim) :: q = (/ 3, 31, -10 /)
END MODULE probData

PROGRAM linmcp
  USE probData
  IMPLICIT NONE
  INTEGER :: n, nnz, i, status
  REAL(8) :: z(probDim), f(probDim), lb(probDim), ub(probDim)

  !DEC$ ATTRIBUTES STDCALL :: pathmain
  !DEC$ ATTRIBUTES REFERENCE :: pathmain
  EXTERNAL :: pathmain

  n = probDim
  nnz = probDim*probDim

  DO i = 1,n
     lb(i) = 1.0
     ub(i) = 10.0
     z(i) = 5.0
  END DO

  CALL pathmain(n, nnz, status, z, f, lb, ub)

  STOP
END PROGRAM linmcp

INTEGER FUNCTION funceval(n,z,f)
  !DEC$ ATTRIBUTES STDCALL :: funceval
  !DEC$ ATTRIBUTES REFERENCE :: funceval
  USE probData
  IMPLICIT NONE
  INTEGER :: n
  REAL(8) :: z(n), f(n)

  INTEGER :: i, j

  DO i = 1,n
     f(i) = q(i)
  END DO
  DO i = 1,n
     DO j = 1,n
        f(i) = f(i) + mat(i,j)*z(j)
     END DO
  END DO
  funceval = 0
  RETURN
END FUNCTION funceval

INTEGER FUNCTION jaceval(n,nnz,z,col,len,row,data)
  !DEC$ ATTRIBUTES STDCALL :: jaceval
  !DEC$ ATTRIBUTES REFERENCE :: jaceval
  USE probData
  IMPLICIT NONE
  INTEGER :: n, nnz
  REAL(8) :: z(n), data(nnz)
  INTEGER :: col(n), len(n), row(nnz)

  INTEGER :: i, j

  DO j = 1,n
     col(j) = (j-1)*probDim + 1
     len(j) = probDim
     DO i = 1,n
        row(i + (j-1)*probDim) = i
     END DO
  END DO

  DO i = 1,probDim
     DO j = 1,probDim
        data(probDim*(j-1)+i) = mat(i,j)
     END DO
  END DO

  jaceval = 0
  RETURN
END FUNCTION jaceval
