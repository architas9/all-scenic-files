param map = localPath('../assets/maps/CARLA/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

EGO_MODEL = "vehicle.tesla.model3"
LEAD_MODEL = "vehicle.audi.tt"

EGO_SPEED = 5
LEAD_SPEED = 3
LANE_CHANGE_TRIGGER_DISTANCE = 12

behavior LeadVehicleBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

behavior EgoAvoidLeadCarsBehavior(rightLaneSection, nearestLeadCar):
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when (distance from self to nearestLeadCar) < LANE_CHANGE_TRIGGER_DISTANCE:
        do LaneChangeBehavior(laneSectionToSwitch=rightLaneSection, target_speed=EGO_SPEED)
        do FollowLaneBehavior(target_speed=EGO_SPEED, laneToFollow=rightLaneSection.lane)

leftmostLaneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec._fasterLane is None and sec._slowerLane is not None:
            leftmostLaneSections.append(sec)

require len(leftmostLaneSections) > 0

leftLaneSection = Uniform(*leftmostLaneSections)
rightLaneSection = leftLaneSection._slowerLane

require rightLaneSection is not None

lead1Spot = new OrientedPoint on leftLaneSection.centerline

lead1 = new Car at lead1Spot,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior()

lead2 = new Car ahead of lead1 by 18,
    with blueprint LEAD_MODEL,
    with behavior LeadVehicleBehavior()

ego = new Car behind lead1 by 40,
    with blueprint EGO_MODEL,
    with behavior EgoAvoidLeadCarsBehavior(rightLaneSection, lead1)
require distance from ego to intersection > 30
require 30 < (distance from ego to lead1) < 50
require 12 < (distance from lead1 to lead2) < 25
require ego can see lead1

terminate after 25 seconds
