param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 5000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
ADV_SPEED = 5
COLLISION_DISTANCE = 3

behavior EgoDriveBehavior():
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior AdversaryMergeRightBehavior(targetLaneSec):
    do FollowLaneBehavior(target_speed=ADV_SPEED) for 1 seconds
    do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=ADV_SPEED)
    do FollowLaneBehavior(target_speed=ADV_SPEED, laneToFollow=targetLaneSec.lane)

lanePairs = []

for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._slowerLane is None and laneSec._fasterLane is not None:
            lanePairs.append((laneSec, laneSec._fasterLane))

require len(lanePairs) > 0

chosenPair = Uniform(*lanePairs)
rightLaneSec = chosenPair[0]
leftLaneSec = chosenPair[1]

egoSpawn = new OrientedPoint on rightLaneSec.centerline
advSpawn = new OrientedPoint on leftLaneSec.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoDriveBehavior()

adversary = new Car at advSpawn,
    with blueprint ADV_MODEL,
    with behavior AdversaryMergeRightBehavior(rightLaneSec)

require 3 < (distance from ego to adversary) < 9

terminate when (distance from ego to adversary) < COLLISION_DISTANCE
terminate when (distance from ego to egoSpawn) > 120
