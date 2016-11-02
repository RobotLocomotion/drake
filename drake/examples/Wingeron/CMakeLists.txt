drake_find_package(avl CONFIG)
drake_find_package(xfoil CONFIG)

drake_add_matlab_test(NAME examples/Wingeron/findTrim REQUIRES avl xfoil OPTIONAL bullet COMMAND findTrim)
drake_add_matlab_test(NAME examples/Wingeron/runPassive REQUIRES avl xfoil OPTIONAL bullet COMMAND runPassive)
