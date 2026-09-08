param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 6
ADV_SPEED = 4
COLLISION_DISTANCE = 3

behavior EgoDriveStraightBehavior():
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior AdversaryCutInBehavior(targetLaneSec):
    do FollowLaneBehavior(target_speed=ADV_SPEED) for 1.5 seconds
    do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=ADV_SPEED)
    do FollowLaneBehavior(target_speed=ADV_SPEED, laneToFollow=targetLaneSec.lane)

candidateLanePairs = []

for lane in network.lanes:
    for sec in lane.sections:
        if sec._fasterLane is not None:
            candidateLanePairs.append((sec, sec._fasterLane))
        if sec._slowerLane is not None:
            candidateLanePairs.append((sec, sec._slowerLane))

require len(candidateLanePairs) > 0

chosenPair = Uniform(*candidateLanePairs)
egoLaneSec = chosenPair[0]
adjacentLaneSec = chosenPair[1]

egoStart = new OrientedPoint on egoLaneSec.centerline
advStart = new OrientedPoint on adjacentLaneSec.centerline

ego = new Car at egoStart,
    with blueprint EGO_MODEL,
    with behavior EgoDriveStraightBehavior()

adversary = new Car at advStart,
    with blueprint ADV_MODEL,
    with behavior AdversaryCutInBehavior(egoLaneSec)

require ego.laneSection is egoLaneSec
require adversary.laneSection is adjacentLaneSec
require 8 < (distance from ego to adversary) < 20
require ego can see adversary

terminate when (distance from ego to adversary) < COLLISION_DISTANCE
terminate when (distance from ego to egoStart) > 140
