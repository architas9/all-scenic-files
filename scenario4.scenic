param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.tesla.model3"
ONCOMING_MODEL = "vehicle.audi.tt"

EGO_SPEED = 3
ONCOMING_SPEED = 7

behavior EgoSlowLeftTurnBehavior(turnTrajectory):
    do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=turnTrajectory)
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior OncomingStraightBehavior(straightTrajectory):
    do FollowTrajectoryBehavior(target_speed=ONCOMING_SPEED, trajectory=straightTrajectory)
    do FollowLaneBehavior(target_speed=ONCOMING_SPEED)

candidatePairs = []

for inter in network.intersections:
    for lane in inter.incomingLanes:
        for egoMan in lane.maneuvers:

            if egoMan.type is ManeuverType.LEFT_TURN:

                # Find the straight maneuver leaving from the ego's approach
                for egoStraightMan in egoMan.startLane.maneuvers:

                    if egoStraightMan.type is ManeuverType.STRAIGHT:

                        # Reverse of this straight maneuver comes from
                        # the opposite side of the intersection
                        for reverseMan in egoStraightMan.reverseManeuvers:

                            if (
                                reverseMan.type is ManeuverType.STRAIGHT
                                and reverseMan in egoMan.conflictingManeuvers
                            ):
                                candidatePairs.append(
                                    (egoMan, reverseMan)
                                )

require len(candidatePairs) > 0

chosenPair = Uniform(*candidatePairs)
egoManeuver = chosenPair[0]
oncomingManeuver = chosenPair[1]

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
oncomingTrajectory = [oncomingManeuver.startLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]

egoStart = new OrientedPoint on egoManeuver.startLane.centerline
oncomingStart = new OrientedPoint on oncomingManeuver.startLane.centerline

ego = new Car at egoStart,
    with blueprint EGO_MODEL,
    with behavior EgoSlowLeftTurnBehavior(egoTrajectory)

oncomingCar = new Car at oncomingStart,
    with blueprint ONCOMING_MODEL,
    with behavior OncomingStraightBehavior(oncomingTrajectory)

require 6 < (distance from ego to egoManeuver.connectingLane) < 22
require 18 < (distance from oncomingCar to oncomingManeuver.connectingLane) < 45
require (distance from ego to oncomingCar) > 15

terminate when (distance from ego to oncomingCar) < 2.5
terminate after 20 seconds
