param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.tesla.model3"
LEAD_MODEL = "vehicle.audi.tt"

EGO_SPEED = 8
LEAD_SPEED = 5
TRAFFIC_MIN_SPEED = 4
TRAFFIC_MAX_SPEED = 9

NUM_TRAFFIC_CARS = 8

behavior EgoNoBrakeBehavior():
    do FollowLaneBehavior(target_speed=EGO_SPEED)

behavior LeadBrakeBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED) for 2 seconds
    take SetBrakeAction(1.0)
    do FollowLaneBehavior(target_speed=0) for 20 seconds

behavior TrafficBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

middleLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec._fasterLane is not None and sec._slowerLane is not None:
            middleLaneSections.append(sec)

require len(middleLaneSections) > 0

egoLaneSec = Uniform(*middleLaneSections)

leadStart = new OrientedPoint on egoLaneSec.centerline

leadCar = new Car at leadStart,
    with blueprint LEAD_MODEL,
    with behavior LeadBrakeBehavior()

ego = new Car behind leadCar by 32,
    with blueprint EGO_MODEL,
    with behavior EgoNoBrakeBehavior()

require ego can see leadCar
require 25 < (distance from ego to leadCar) < 40

allLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        allLaneSections.append(sec)

trafficCars = []
for i in range(NUM_TRAFFIC_CARS):
    trafficLaneSec = Uniform(*allLaneSections)
    trafficSpot = new OrientedPoint on trafficLaneSec.centerline
    trafficSpeed = Uniform(TRAFFIC_MIN_SPEED, TRAFFIC_MAX_SPEED)
    trafficModel = Uniform(
        "vehicle.lincoln.mkz_2017",
        "vehicle.nissan.patrol",
        "vehicle.chevrolet.impala",
        "vehicle.bmw.grandtourer",
        "vehicle.mercedes.coupe"
    )

    trafficCar = new Car at trafficSpot,
        with blueprint trafficModel,
        with behavior TrafficBehavior(trafficSpeed)

    require (distance from trafficCar to ego) > 18
    require (distance from trafficCar to leadCar) > 18

    trafficCars.append(trafficCar)

terminate when (distance from ego to leadCar) < 5
terminate after 25 seconds
