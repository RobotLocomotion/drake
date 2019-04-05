from pydrake.autodiffutils import AutoDiffXd
from pydrake.symbolic import Expression
from pydrake.systems.framework import BasicVector, BasicVector_


def MakeNamedVectorType(classname, variable_names, T=None):
    """
    Creates a subclass of BasicVector with named getters and setters for the
    individual elements.

    Args:
        classname: A string name for the new class
        variable_names: A list of strings to name the variables.  The size
            of the resulting BasicVector is len(variable_names).
        T: (Advanced) A python type for the templated class.  Most users
            should leave this as None, or call MakeNamedVectorScalarTypes.

    Example:
        MyVector = MakeNamedVectorType('MyVector', ['x','y','z'])
        a = MyVector()
        a.set_z(0.3)
        a.z()  # To retrieve the value.
        b = MyVector([1., 2., 3.])  # To initialize with values

    Note:
        This method enables making and immediately using named vectors in
        Python.  It is independent from the 'named vector_gen' which codegens
        types that can be used in C++.
    """
    def vector_init(self, num_vars, data, BaseClass):
        if data is None:
            BaseClass.__init__(self, num_vars)
        else:
            assert len(data) == num_vars, "data must have length " \
                                          + str(num_vars)
            BaseClass.__init__(self, data)

    if T is None:
        BaseClass = BasicVector
    else:
        BaseClass = BasicVector_[T]

    attributes = {"__init__":
                  lambda self, data=None: vector_init(self,
                                                      len(variable_names),
                                                      data,
                                                      BaseClass)}
    for i in range(len(variable_names)):
        # Note: need the double lambda to bind the current i.
        attributes[variable_names[i]] = \
            (lambda i: lambda self: self.GetAtIndex(i))(i)
        attributes["set_" + variable_names[i]] = \
            (lambda i: lambda self, val: self.SetAtIndex(i, val))(i)

    return type(classname, (BaseClass,), attributes)


def MakeNamedVectorScalarTypes(classname, variable_names):
    """
    Creates the dictionary of the standard Drake scalar types for a subclass
    of BasicVector with named getters and setters for the individual
    elements.  (see MakeNamedVectorType).

    Example:
        MyVector_ = MakeNamedVectorScalarTypes('MyVector', ['x','y','z'])
        a = MyVector_[float]()
        b = MyVector_[AutoDiffXd]()
        c = MyVector_[Expression](np.zeros(3))
    """
    vector_ = dict()
    for T in [float, AutoDiffXd, Expression]:
        vector_[T] = MakeNamedVectorType(classname + "_", variable_names, T)
    return vector_
