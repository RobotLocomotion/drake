import lcm, math, random, sys, time
from drake.lcmt_lidar_field import lcmt_lidar_field as lcm_field
from drake.lcmt_lidar_data import lcmt_lidar_data as lcm_msg

#------------------------------------------------------------------------------
def add_field(msg, field, data):
    field.values = data;
    field.num_values = len(data)

    msg.fields.append(field)

#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if __name__ == '__main__':
    comm = lcm.LCM()

    position_field = lcm_field()
    position_field.id = lcm_msg.POSITION

    range_field = lcm_field()
    range_field.id = lcm_msg.RANGE

    intensity_field = lcm_field()
    intensity_field.id = lcm_msg.INTENSITY

    scan_field = lcm_field()
    scan_field.id = lcm_msg.SCAN

    echo_field = lcm_field()
    echo_field.id = lcm_msg.ECHO

    t = 0
    while True:
        t = (t + 0.015) % (2.0 * math.pi)

        range_data = []
        position_data = []
        intensity_data = []
        scan_data = []
        echo_data = []

        msg = lcm_msg()
        for k in xrange(16):
            for n in range(random.choice([1,1,1,1,1,2,2,2,3,3,4])):
                p = k / 40.0
                r = random.uniform(0, 500)
                range_data.append(r)
                position_data.append(r * math.sin(t) * math.cos(p))
                position_data.append(r * math.cos(t) * math.cos(p))
                position_data.append(r * math.sin(p))
                intensity_data.append(random.random())
                scan_data.append(k)
                echo_data.append(n + 1)

        add_field(msg, position_field, position_data)
        add_field(msg, range_field, range_data)
        add_field(msg, intensity_field, intensity_data)
        add_field(msg, scan_field, scan_data)
        add_field(msg, echo_field, echo_data)
        msg.num_fields = len(msg.fields)

        msg.timestamp = time.time() * 1e6
        msg.scan_angle = t
        msg.scan_direction = False

        comm.publish("DRAKE_POINTCLOUD_LIDAR_EXAMPLE_RANDOM", msg.encode())
