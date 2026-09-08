param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 5000

EGO_MODEL = "vehicle.tesla.model3"
LEAD_MODEL = "vehicle.carlamotors.carlacola"
ADVERSARY_MODEL = "vehicle.audi.tt"

EGO_SPEED = 6
LEAD_SPEED = 3
ADVERSARY_INITIAL_SPEED = 3
ADVERSARY_ACCEL_SPEED = 7
COLLISION_DISTANCE = 3

behavior EgoPassSlowVehicleBehavior(targetLaneSec):
    do FollowLaneBehavior(target_speed=EGO_SPEED) for 1.5 seconds
    do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=EGO_SPEED)
    do FollowLaneBehavior(target_speed=EGO_SPEED, laneToFollow=targetLaneSec.lane)

behavior SlowLeadVehicleBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

behavior AdversaryAcceleratesInTargetLaneBehavior(targetLane):
    do FollowLaneBehavior(target_speed=ADVERSARY_INITIAL_SPEED, laneToFollow=targetLane) for 1 seconds
    do FollowLaneBehavior(target_speed=ADVERSARY_ACCEL_SPEED, laneToFollow=targetLane)

candidateLanePairs = []

for lane in network.lanes:
    for sec in lane.sections:
        if sec._fasterLane is not None:
            candidateLanePairs.append((sec, sec._fasterLane))

require len(candidateLanePairs) > 0

chosenPair = Uniform(*candidateLanePairs)
egoLaneSec = chosenPair[0]
targetLaneSec = chosenPair[1]

egoSpawn = new OrientedPoint on egoLaneSec.centerline
adversarySpawn = new OrientedPoint on targetLaneSec.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoPassSlowVehicleBehavior(targetLaneSec)

leadVehicle = new Car ahead of ego by 20,
    with blueprint LEAD_MODEL,
    with behavior SlowLeadVehicleBehavior()

adversary = new Car at adversarySpawn,
    with blueprint ADVERSARY_MODEL,
    with behavior AdversaryAcceleratesInTargetLaneBehavior(targetLaneSec.lane)

require 8 < (distance from ego to adversary) < 24
require ego can see leadVehicle
require ego can see adversary

terminate when (distance from ego to adversary) < COLLISION_DISTANCE
terminate when (distance from ego to leadVehicle) > 40
terminate when (distance from ego to egoSpawn) > 140
