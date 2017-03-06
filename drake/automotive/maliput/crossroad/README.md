# Description

Crossroad is an implementation of Maliput's API that allows users to instantiate
an 'X'-shaped multi-lane crossroad. The crossroad has two segments, one 
horizontal and the other vertical. All lanes in the same segment are straight 
and parallel. The ends of each lane are connected together via a "magical
loop" that results in vehicles traveling on the Crossroad's lanes 
instantaneously teleporting from one end of the lane to the opposite end of the 
lane.

The number of lanes and their lengths, widths, and shoulder widths are all user
specifiable. For example, the number of horizontal lanes can be specified by 
passing the command line flag --num_horizontal_crossroad_lanes. For more 
customized flags, see `automotive_demo.cc` in /drake/automotive.
