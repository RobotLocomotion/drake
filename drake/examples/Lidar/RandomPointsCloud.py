import lcm, math, random, sys, time
from drake.lcmt_lidar_point import lcmt_lidar_point as lcm_point
from drake.lcmt_lidar_data import lcmt_lidar_data as lcm_msg

if __name__ == '__main__':
    comm = lcm.LCM()

    t = 0
    while True:
        t = (t + 0.015) % (2.0 * math.pi)

        msg = lcm_msg()
        for k in xrange(16):
            for n in range(random.choice([1,1,1,1,1,2,2,2,3,3,4])):
                p = lcm_point()
                r = random.uniform(0, 500)
                p.x = r * math.sin(t)
                p.y = r * math.cos(t)
                p.z = r * k / 80
                p.intensity = random.random()
                p.scan = k
                p.echo = n + 1
                msg.points.append(p)

        msg.timestamp = time.time() * 1e6
        msg.num_points = len(msg.points)
        msg.scan_angle = t
        msg.scan_direction = False

        comm.publish("DRAKE_POINTCLOUD_LIDAR_EXAMPLE_RANDOM", msg.encode())
