param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

param maxIterations = 3000

EGO_MODEL = "vehicle.tesla.model3"
ADVERSARY_MODEL = "vehicle.audi.tt"

EGO_SPEED = 7
ADVERSARY_SPEED = 5
INITIAL_GAP = 24
CLOSE_DISTANCE = 4

behavior EgoDriveStraightBehavior():
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior SuddenBrakeBehavior():
    do FollowLaneBehavior(target_speed=ADVERSARY_SPEED) for 2 seconds
    while True:
        take SetBrakeAction(1.0)

laneSections = []

for lane in network.lanes:
    for sec in lane.sections:
        laneSections.append(sec)

require len(laneSections) > 0

egoLaneSection = Uniform(*laneSections)
egoSpawn = new OrientedPoint on egoLaneSection.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoDriveStraightBehavior()

adversary = new Car ahead of ego by INITIAL_GAP,
    with blueprint ADVERSARY_MODEL,
    with behavior SuddenBrakeBehavior()

require ego can see adversary

terminate when (distance from ego to adversary) < CLOSE_DISTANCE
terminate when (distance from ego to egoSpawn) > 120
terminate after 20 seconds
