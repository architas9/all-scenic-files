param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 6
ADV_SPEED = 5
COLLISION_DISTANCE = 3

behavior EgoStraightBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=EGO_SPEED)
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior AdversaryLeftTurnBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=ADV_SPEED)
    do FollowLaneBehavior(target_speed=ADV_SPEED)

candidatePairs = []

for inter in network.intersections:
    for egoLane in inter.incomingLanes:
        for egoStraight in egoLane.maneuvers:
            if egoStraight.type is ManeuverType.STRAIGHT and egoStraight.connectingLane is not None:
                for reverseStraight in egoStraight.reverseManeuvers:
                    if reverseStraight.type is ManeuverType.STRAIGHT:
                        advLane = reverseStraight.startLane
                        if advLane in inter.incomingLanes:
                            for advLeft in advLane.maneuvers:
                                if (
                                    advLeft.type is ManeuverType.LEFT_TURN
                                    and advLeft.connectingLane is not None
                                    and advLeft in egoStraight.conflictingManeuvers
                                ):
                                    candidatePairs.append((inter, egoStraight, advLeft))

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
    with behavior EgoStraightBehavior(egoTrajectory)

adversary = new Car at advStart,
    with blueprint ADV_MODEL,
    with behavior AdversaryLeftTurnBehavior(advTrajectory)

require 12 < (distance from ego to intersection) < 45
require 12 < (distance from adversary to intersection) < 45
require abs((distance from ego to intersection) - (distance from adversary to intersection)) < 10

terminate when (distance from ego to adversary) < COLLISION_DISTANCE
terminate when (distance from ego to intersection) > 70
terminate after 20 seconds
