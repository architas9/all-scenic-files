param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
ADV_SPEED = 4
START_GAP = 20
STOP_AFTER_TIME = 4
STOPPED_DISTANCE = 4

behavior EgoFollowBehavior():
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior AdversaryStopInLaneBehavior():
    do FollowLaneBehavior(target_speed=ADV_SPEED) for STOP_AFTER_TIME seconds
    while True:
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)

laneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        laneSections.append(sec)

require len(laneSections) > 0

chosenLaneSec = Uniform(*laneSections)

egoStart = new OrientedPoint on chosenLaneSec.centerline

ego = new Car at egoStart,
    with blueprint EGO_MODEL,
    with behavior EgoFollowBehavior()

adversary = new Car ahead of ego by START_GAP,
    with blueprint ADV_MODEL,
    with behavior AdversaryStopInLaneBehavior()

require ego.laneSection is chosenLaneSec
require adversary.laneSection is chosenLaneSec
require 12 < (distance from ego to adversary) < 28

terminate when (distance from ego to adversary) < STOPPED_DISTANCE
terminate after 20 seconds
