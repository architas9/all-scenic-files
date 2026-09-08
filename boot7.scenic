param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 5000

EGO_MODEL = "vehicle.tesla.model3"
ADVERSARY_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
ADVERSARY_SPEED = 6
CLOSE_DISTANCE = 3.5

behavior DriveStraightThroughIntersection(traj, speed):
    do FollowTrajectoryBehavior(trajectory=traj, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

candidatePairs = []

for inter in network.intersections:
    if inter.is4Way:
        for egoLane in inter.incomingLanes:
            egoStraightManeuvers = []
            egoRightTurnManeuvers = []

            for egoMan in egoLane.maneuvers:
                if egoMan.type is ManeuverType.STRAIGHT:
                    egoStraightManeuvers.append(egoMan)
                if egoMan.type is ManeuverType.RIGHT_TURN:
                    egoRightTurnManeuvers.append(egoMan)

            for egoStraight in egoStraightManeuvers:
                for egoRight in egoRightTurnManeuvers:
                    for reverseRight in egoRight.reverseManeuvers:
                        rightApproachLane = reverseRight.startLane
                        for advMan in rightApproachLane.maneuvers:
                            if advMan.type is ManeuverType.STRAIGHT:
                                if advMan in egoStraight.conflictingManeuvers:
                                    candidatePairs.append((inter, egoStraight, advMan))

require len(candidatePairs) > 0

chosenPair = Uniform(*candidatePairs)

intersection = chosenPair[0]
egoManeuver = chosenPair[1]
adversaryManeuver = chosenPair[2]

egoTrajectory = [
    egoManeuver.startLane,
    egoManeuver.connectingLane,
    egoManeuver.endLane
]

adversaryTrajectory = [
    adversaryManeuver.startLane,
    adversaryManeuver.connectingLane,
    adversaryManeuver.endLane
]

egoSpawn = new OrientedPoint on egoManeuver.startLane.centerline
adversarySpawn = new OrientedPoint on adversaryManeuver.startLane.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior DriveStraightThroughIntersection(egoTrajectory, EGO_SPEED)

adversary = new Car at adversarySpawn,
    with blueprint ADVERSARY_MODEL,
    with behavior DriveStraightThroughIntersection(adversaryTrajectory, ADVERSARY_SPEED)

require 18 < (distance from ego to intersection) < 38
require 10 < (distance from adversary to intersection) < 30
require (distance from adversary to intersection) < (distance from ego to intersection)
require (distance from ego to adversary) > 10

terminate when (distance from ego to adversary) < CLOSE_DISTANCE
terminate when (distance from ego to intersection) > 80
