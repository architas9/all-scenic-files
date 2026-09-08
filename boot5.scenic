param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
ADV_SPEED = 6
CONFLICT_DISTANCE = 3.5

behavior EgoLeftTurnBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=EGO_SPEED)
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior OncomingStraightBehavior(traj):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=ADV_SPEED)
    do FollowLaneBehavior(target_speed=ADV_SPEED)

candidatePairs = []

for inter in network.intersections:
    for egoLane in inter.incomingLanes:
        egoStraightManeuvers = []
        for man in egoLane.maneuvers:
            if man.type is ManeuverType.STRAIGHT and man.connectingLane is not None:
                egoStraightManeuvers.append(man)

        oppositeStraightManeuvers = []
        for straightMan in egoStraightManeuvers:
            for reverseMan in straightMan.reverseManeuvers:
                if reverseMan.type is ManeuverType.STRAIGHT and reverseMan.connectingLane is not None:
                    if reverseMan.startLane in inter.incomingLanes:
                        oppositeStraightManeuvers.append(reverseMan)

        for egoLeft in egoLane.maneuvers:
            if egoLeft.type is ManeuverType.LEFT_TURN and egoLeft.connectingLane is not None:
                for oncomingStraight in oppositeStraightManeuvers:
                    if oncomingStraight in egoLeft.conflictingManeuvers:
                        candidatePairs.append((inter, egoLeft, oncomingStraight))

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
    with behavior EgoLeftTurnBehavior(egoTrajectory)

adversary = new Car at advStart,
    with blueprint ADV_MODEL,
    with behavior OncomingStraightBehavior(advTrajectory)

require 12 < (distance from ego to intersection) < 45
require 12 < (distance from adversary to intersection) < 45
require abs((distance from ego to intersection) - (distance from adversary to intersection)) < 12

terminate when (distance from ego to adversary) < CONFLICT_DISTANCE
terminate after 20 seconds
