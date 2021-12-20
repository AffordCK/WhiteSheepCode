
logger = Logger();
fprintf("[%s]: Starting Log...\n", datestr(now));

env = Env();

[env, vehicleSet, vehiclesTrajectory] = env.load_scenario("Scenario/05.mat", 5);

x = [-10, 0: 100: 500];
y = zeros(size(x));
waypoint = [x', y'];

env = env.load_road(waypoint, vehicleSet, vehiclesTrajectory);

startState = env.RoadInstance.get_vehicle_frenetstate(1);

endState = [env.Conf.MaxS, env.Conf.MaxSpeed, 0, 1.8, 0, 0];

T = 2 * (endState(1) - startState(1)) / (endState(2) + startState(2));

plannResult = env.PlannerInstance.get_trajectory(startState, endState, T, env.Conf.TimeResolution, 1);

env.PlannerInstance.check_trajectory(plannResult.trajectory);

globalTrajectory = env.RoadInstance.frenet_to_global_trajectory(plannResult.trajectory);

env.RoadInstance = env.RoadInstance.add_vistor(waypoint);

env.RoadInstance.move_along_trajectory_vistor(globalTrajectory, 0);

fprintf("[%s]: Stopping Log...\n", datestr(now));
logger.stop_log();