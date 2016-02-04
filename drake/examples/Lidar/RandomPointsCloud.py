import lcm, math, random, sys, time
from drake.lcmt_lidar_point import lcmt_lidar_point as lcm_point
from drake.lcmt_lidar_data import lcmt_lidar_data as lcm_msg

if __name__ == '__main__':
    comm = lcm.LCM()

    while True:
        msg = lcm_msg()
        msg.num_points = random.randint(100, 1000)
        for k in xrange(msg.num_points):
            p = lcm_point()
            r = random.uniform(0, 500)
            t = random.uniform(0, 2*math.pi)
            p.x = r * math.sin(t)
            p.y = r * math.cos(t)
            p.z = random.uniform(0, 50)
            p.intensity = random.random()
            p.scan = k % 16
            p.echo = random.choice([0,0,0,0,0,1,1,1,2,2,3])
            msg.points.append(p)

        comm.publish("DRAKE_POINTCLOUD_LIDAR_EXAMPLE_RANDOM", msg.encode())
        time.sleep(0.1)
