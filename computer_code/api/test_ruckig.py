from ruckig import InputParameter, OutputParameter, Result, Ruckig
import copy


num_objects = 1

max_vel = [1.0, 1.0, 1.0]
max_accel = [1.0, 1.0, 1.0]
max_jerk = [0.5, 0.5, 0.5]
# timestep = 0.1
timestep = 1



start_pos = [0.3, 0.3, 0.3]
end_pos = [0.4, 0.6, 1]

# waypoints = [
# ]

waypoints = [[0.35, 0.5, 0.5]]


def plan_trajectory(start_pos, end_pos, waypoints, max_vel, max_accel, max_jerk, timestep):
    otg = Ruckig(3*num_objects, timestep, len(waypoints))  # DoFs, timestep, number of waypoints
    inp = InputParameter(3*num_objects)
    out = OutputParameter(3*num_objects, len(waypoints))

    inp.current_position = start_pos
    inp.current_velocity = [0,0,0]*num_objects
    inp.current_acceleration = [0,0,0]*num_objects

    inp.target_position = end_pos
    inp.target_velocity = [0,0,0]*num_objects
    inp.target_acceleration = [0,0,0]*num_objects

    inp.intermediate_positions = waypoints

    inp.max_velocity = max_vel*num_objects
    inp.max_acceleration = max_accel*num_objects
    inp.max_jerk = max_jerk*num_objects

    setpoints = []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)
        setpoints.append(copy.copy(out.new_position))
        out.pass_to_input(inp)

    return setpoints


setpoints = plan_trajectory(start_pos, end_pos, waypoints, max_vel, max_accel, max_jerk, timestep)
print("setpoints", setpoints)
