function robotiq_benchmark

sim_time_full = timeit(@()robotiq_sim('robotiq.urdf'))
sim_time_limp = timeit(@()robotiq_sim('robotiq_limp.urdf'))
sim_time_springs = timeit(@()robotiq_sim('robotiq_springs.urdf'))
sim_time_friction_springs = timeit(@()robotiq_sim('robotiq_springs_damping.urdf'))

end

