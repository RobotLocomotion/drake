import numpy as np


def _check(result, expected):
    assert (result == expected).all(), repr(result)


def main():
    import sample_module as dut

    # Dense 2-d matrix accepts either ndarray or primitive lists.
    prim22 = [[1.0, 2.0], [3.0, 4.0]]
    arr22 = np.array(prim22)
    _check(dut.copy_dense_mat(arr22), arr22)
    _check(dut.copy_dense_mat(prim22), arr22)
    _check(dut.copy_dense_mat_ref(arr22), arr22)
    _check(dut.copy_dense_mat_ref(prim22), arr22)

    # Dense 1-d vector accepts either ndarray or primitive lists.
    prim3 = [1.0, 2.0, 3.0]
    arr3 = np.array(prim3)
    _check(dut.copy_dense_vec(arr3), arr3)
    _check(dut.copy_dense_vec(prim3), arr3)
    _check(dut.copy_dense_vec_ref(arr3), arr3)
    _check(dut.copy_dense_vec_ref(prim3), arr3)

    # Dense 2-d matrix accepts the same 1-d vectors as in the prior paragraph
    # (prim3 and arr3). The Eigen::Matrix ends up receiving the 1-d vector as a
    # 2-d column vector, as indicated by the return type being shaped as arr31.
    prim31 = [[1.0], [2.0], [3.0]]
    arr31 = np.array(prim31)
    _check(dut.copy_dense_mat(arr3), arr31)
    _check(dut.copy_dense_mat(prim3), arr31)
    _check(dut.copy_dense_mat_ref(arr3), arr31)
    _check(dut.copy_dense_mat_ref(prim3), arr31)

    # Dense 1-d vector accepts 2-d column vectors.
    _check(dut.copy_dense_vec(arr31), arr3)
    _check(dut.copy_dense_vec(prim31), arr3)
    _check(dut.copy_dense_vec_ref(arr31), arr3)
    _check(dut.copy_dense_vec_ref(prim31), arr3)

    # Data that numpy doesn't know how to convert (e.g., ragged lists) are
    # rejected with a cast error. N.B. This must be type_caster error, not the
    # numpy error -- in case the function was overloaded, we need to allow any
    # other type_casters their chance of accepting this data.
    try:
        dut.copy_dense_mat([[1.0, 2.0], [3.0]])
        error = None
    except TypeError as e:
        error = str(e)
    assert "copy_dense_mat(): incompatible function arguments" in error, error

    # Basic sparse and dense casting of numpy and scipy.
    sparse22 = dut.dense_to_sparse(prim22)
    dense22 = dut.sparse_to_dense(sparse22)
    assert (dense22 == arr22).all(), repr((sparse22, dense22))


assert __name__ == "__main__"
main()
