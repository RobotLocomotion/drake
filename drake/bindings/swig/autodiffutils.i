%module(package="pydrake") autodiffutils


%include "exception_helper.i"
%include <eigen.i>
%include <autodiff.i>

#ifdef SWIGPYTHON3
%rename(__truediv__)     *::operator/;
#endif

%template(AutoDiffVectorDynamic) AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, 1>;
%template(AutoDiffVectorMax73) AutoDiffWrapper<Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 73>, Eigen::Dynamic, 1>;
%template(AutoDiffMatrixDynamic) AutoDiffWrapper<Eigen::VectorXd, Eigen::Dynamic, Eigen::Dynamic>;

#ifdef SWIGPYTHON
%pythoncode %{
import numpy

def toAutoDiff(value, derivatives=None):
    value = numpy.asarray(value)
    if derivatives is None:
        derivatives = numpy.eye(value.size, value.size)
    else:
        derivatives = numpy.asarray(derivatives)
        if derivatives.ndim < 2:
            derivatives = derivatives.reshape((-1,1))
    if value.ndim == 0:
        value = value.reshape((1,))
    if value.ndim < 2 or value.shape[1] == 1:
        if derivatives.shape[1] <= 73:
            return AutoDiffVectorMax73(value, derivatives)
        else:
            return AutoDiffVectorDynamic(value, derivatives)
    else:
        return AutoDiffMatrixDynamic(value, derivatives)

%}
#endif
