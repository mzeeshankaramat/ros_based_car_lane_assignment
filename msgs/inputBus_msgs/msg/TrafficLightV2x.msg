uint64 stationID                      # (long long) ID of the V2X message sender (e.g. RSU, vehicle)
uint64 intersectionID                 # (long long) ID of the intersection which consists of multiple signal groups
uint32 signalGroupID                  # the group id is used to map to lists of lanes(and their descriptions) which this MovementState data applies to
# HAD_Points::SphericalPoint          intersectionRefPoint;           # Reference point of the intersection in geographic coordinate system (lat,lon, alt)             
# sequence<short>                     drivableLaneIDs;                # IDs of the driveable lanes for the traffic light
# sequence<MovementEvent>             movementEvents;                 # the states of the traffic light  