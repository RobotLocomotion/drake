# This test is simply to confirm that nested LCM-generated python messages are
# code-gen'd correctly, and can be imported at will.  If something isn't
# working, it will raise an exception during instantiation or construction.

from drake import lcmt_robot_plan, lcmt_robot_state

outer = lcmt_robot_plan()
outer.num_states = 1
outer.plan.append(lcmt_robot_state())
outer.encode()
