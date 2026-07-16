import numpy as np


def main():
    import sample_module

    x = np.array([[1.0, 2.0], [3.0, 4.0]])
    y = sample_module.dense_to_sparse(x)
    z = sample_module.sparse_to_dense(y)
    assert (x == z).all(), f"{x!r} {z!r}"


assert __name__ == "__main__"
main()
