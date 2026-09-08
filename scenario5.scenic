param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 5000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 6
ADV_SPEED = 5
COLLISION_DISTANCE = 3.0

behavior EgoStraightBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior OncomingLeftTurnBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=ADV_SPEED, trajectory=trajectory)
    do FollowLaneBehavior(target_speed=ADV_SPEED)

oncomingLeftChoices = []

for inter in network.intersections:
    for egoLane in inter.incomingLanes:
        egoStraightManeuvers = []
        egoRightManeuvers = []

        for man in egoLane.maneuvers:
            if man.type == ManeuverType.STRAIGHT:
                egoStraightManeuvers.append(man)
            if man.type == ManeuverType.RIGHT_TURN:
                egoRightManeuvers.append(man)

        for egoStraight in egoStraightManeuvers:
            for egoRight in egoRightManeuvers:
                for advMan in egoStraight.conflictingManeuvers:
                    if advMan.type == ManeuverType.LEFT_TURN and advMan.endLane is egoRight.endLane:
                        oncomingLeftChoices.append((inter, egoStraight, advMan))

if len(oncomingLeftChoices) == 0:
    for inter in network.intersections:
        for egoLane in inter.incomingLanes:
            for egoMan in egoLane.maneuvers:
                if egoMan.type == ManeuverType.STRAIGHT:
                    for advMan in egoMan.conflictingManeuvers:
                        if advMan.type == ManeuverType.LEFT_TURN:
                            oncomingLeftChoices.append((inter, egoMan, advMan))

assert len(oncomingLeftChoices) > 0, 'No suitable straight-through and oncoming-left-turn conflict found.'

choice = Uniform(*oncomingLeftChoices)

intersection = choice[0]
egoManeuver = choice[1]
advManeuver = choice[2]

egoTrajectory = [egoManeuver.startLane, egoManeuver.connectingLane, egoManeuver.endLane]
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

egoSpawn = new OrientedPoint on egoManeuver.startLane.centerline
advSpawn = new OrientedPoint on advManeuver.startLane.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoStraightBehavior(egoTrajectory)

adversary = new Car at advSpawn,
    with blueprint ADV_MODEL,
    with behavior OncomingLeftTurnBehavior(advTrajectory)

require ego can see adversary

terminate when (distance from ego to adversary) < COLLISION_DISTANCE
terminate when (distance from ego to egoSpawn) > 120
