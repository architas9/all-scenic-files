param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 3.5
ADV_SPEED = 7

behavior EgoLeftTurnFromSideRoadBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=EGO_SPEED)
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior FastMainRoadBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=ADV_SPEED)
    do FollowLaneBehavior(target_speed=ADV_SPEED)

candidatePairs = []

for inter in network.intersections:
    for egoLane in inter.incomingLanes:
        for egoManeuver in egoLane.maneuvers:
            if egoManeuver.type is ManeuverType.LEFT_TURN and egoManeuver.connectingLane is not None:
                for advManeuver in egoManeuver.conflictingManeuvers:
                    if advManeuver.type is ManeuverType.STRAIGHT and advManeuver.connectingLane is not None:
                        candidatePairs.append((inter, egoManeuver, advManeuver))

require len(candidatePairs) > 0

chosenPair = Uniform(*candidatePairs)

intersection = chosenPair[0]
egoManeuver = chosenPair[1]
advManeuver = chosenPair[2]

egoTrajectory = [
    egoManeuver.startLane,
    egoManeuver.connectingLane,
    egoManeuver.endLane
]

advTrajectory = [
    advManeuver.startLane,
    advManeuver.connectingLane,
    advManeuver.endLane
]

egoStart = new OrientedPoint on egoManeuver.startLane.centerline
advStart = new OrientedPoint on advManeuver.startLane.centerline

ego = new Car at egoStart,
    with blueprint EGO_MODEL,
    with behavior EgoLeftTurnFromSideRoadBehavior(egoTrajectory)

adversary = new Car at advStart,
    with blueprint ADV_MODEL,
    with behavior FastMainRoadBehavior(advTrajectory)

require 8 < (distance from ego to intersection) < 28
require 25 < (distance from adversary to intersection) < 65

terminate when (distance from ego to adversary) < 4
terminate when (distance from adversary to intersection) > 55
terminate after 18 seconds
