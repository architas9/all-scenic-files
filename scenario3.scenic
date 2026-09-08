param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.tesla.model3"
OTHER_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
OTHER_SPEED = 3
COLLISION_TRIGGER_DISTANCE = 8

behavior EgoMergeRightBehavior(rightLaneSec):
    do FollowLaneBehavior(target_speed=EGO_SPEED) for 1.5 seconds
    do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=EGO_SPEED)
    do FollowLaneBehavior(target_speed=EGO_SPEED, laneToFollow=rightLaneSec.lane)

behavior OtherMergeRightBehavior(rightLaneSec):
    try:
        do FollowLaneBehavior(target_speed=OTHER_SPEED)
    interrupt when (distance from self to ego) < COLLISION_TRIGGER_DISTANCE:
        do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=OTHER_SPEED)
        do FollowLaneBehavior(target_speed=OTHER_SPEED, laneToFollow=rightLaneSec.lane)

candidateLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec._slowerLane is not None:
            candidateLaneSections.append(sec)

require len(candidateLaneSections) > 0

startLaneSec = Uniform(*candidateLaneSections)
rightLaneSec = startLaneSec._slowerLane

otherSpot = new OrientedPoint on startLaneSec.centerline

otherVehicle = new Car at otherSpot,
    with blueprint OTHER_MODEL,
    with behavior OtherMergeRightBehavior(rightLaneSec)

ego = new Car behind otherVehicle by 18,
    with blueprint EGO_MODEL,
    with behavior EgoMergeRightBehavior(rightLaneSec)

require ego can see otherVehicle
require 12 < (distance from ego to otherVehicle) < 25

terminate when (distance from ego to otherVehicle) < 4
terminate after 25 seconds
