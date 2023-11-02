/*
* @filename    cLaneAssignment.cpp
* @author      Zeeshan Karamat
*/

#include <lane_assignment/cLaneAssignmnet.h>
#include <limits>
#include <math.h>

#define PI 3.14159265

//#################################################################################################
   int LanePosToLaneAssignmnet(LanePosition ePosition)
   {
      switch(ePosition)
      {
         case LanePosition::UNDEFINED:
            return LaneAssignmnetPosition::LA_UNKNOWN;
         case LanePosition::EGO_LEFT_LEFT:
            return LaneAssignmnetPosition::LA_LEFT_LEFT_LANE;
         case LanePosition::EGO_LEFT:
            return LaneAssignmnetPosition::LA_LEFT_LANE;
         case LanePosition::EGO_RIGHT:
            return LaneAssignmnetPosition::LA_RIGHT_LANE;
         case LanePosition::EGO_RIGHT_RIGHT:
            return LaneAssignmnetPosition::LA_RIGHT_RIGHT_LANE;
      }
   }
//#################################################################################################
   std::string LaneEnumToString(int8_t ePosition)
   {
      switch(ePosition)
      {
         case LaneAssignmnetPosition::LA_UNKNOWN:
            return "UNKNOWN";
         case LaneAssignmnetPosition::LA_LEFT_LEFT_LANE:
            return "EGO LEFT LEFT";
         case LaneAssignmnetPosition::LA_LEFT_LANE:
            return "EGO LEFT";
         case LaneAssignmnetPosition::LA_EGO_LANE:
            return "EGO";
         case LaneAssignmnetPosition::LA_RIGHT_LANE:
            return "EGO RIGHT";
         case LaneAssignmnetPosition::LA_RIGHT_RIGHT_LANE:
            return "EGO RIGHT RIGHT";
      }
   }

//#################################################################################################
   cLaneAssignment::cLaneAssignment(const ros::NodeHandle& publicHandle,
                                    const ros::NodeHandle& privateHandle):
                                    m_oPublicHandle(publicHandle),
                                    m_oPrivateHandle(privateHandle)
   {
      std::string strObjectListTopicName = DEFAULT_OBJECT_LIST_TOPIC_NAME;
      std::string strLanesTopicName = DEFAULT_LANE_TOPIC_NAME;
      std::string strOutputTopicName = DEFAULT_OUTPUT_TOPIC_NAME;

      m_oPrivateHandle.getParam(OBJECT_INPUT_TOPIC_PARAM, strObjectListTopicName);
      m_oPrivateHandle.getParam(LANE_INPUT_TOPIC_PARAM, strLanesTopicName);

      m_oPrivateHandle.getParam(OBJECT_OUTPUT_TOPIC_PARAM, strOutputTopicName);

      m_oLaneSubscriber = m_oPublicHandle.subscribe(
          strLanesTopicName, QUEUE_SIZE, &cLaneAssignment::LaneSubscriberCallback, this);

      m_oObjectSubscriber = m_oPublicHandle.subscribe(
          strObjectListTopicName, QUEUE_SIZE, &cLaneAssignment::ObjectSubscriberCallback, this);

      m_oPublisher = m_oPublicHandle.advertise<outputBus_msgs::SensorDetectedObjectList>(strOutputTopicName, QUEUE_SIZE);
   }

//#################################################################################################
   void cLaneAssignment::LaneSubscriberCallback(
                              const boost::shared_ptr<inputBus_msgs::LaneMarkingArray const>& pLaneMsg)
   {
      m_lock.lock();
      PopulateLanePositionMap(*pLaneMsg);
      m_lock.unlock();
   }
//#################################################################################################
   void cLaneAssignment::ObjectSubscriberCallback(
                  const boost::shared_ptr<inputBus_msgs::SensorDetectedObjectList const>& pObjMsg)
   {
      inputBus_msgs::SensorDetectedObjectList oObjectData;
      m_lock.lock();
      oObjectData = *pObjMsg;
      AssignLaneToObjects(convert(oObjectData));
      m_lock.unlock();
   }

//#################################################################################################
   // This function get two points of lane and object point and return object position left or right
   ObjectPosion cLaneAssignment::GetObjectPosition(geometry_msgs::Point srcPoint,
                                                   geometry_msgs::Point firstPoint,
                                                   geometry_msgs::Point secondPoint)
   {
      auto x1 = firstPoint.x;
      auto y1 = firstPoint.y;
      auto x2 = secondPoint.x;
      auto y2 = secondPoint.y;

      // get slope of line by these two points
      double slopeInDegree = atan2((y2 - y1), (x2 - x1)) * 180 / PI;

      // check for slope to pick x or y direction criteria for object position
      if(slopeInDegree > 60 && slopeInDegree < 120)
      {
         if (firstPoint.x < srcPoint.x)
            return ObjectPosion::RIGHT;
         return ObjectPosion::LEFT;
      }
      else
      {
          if (firstPoint.y > srcPoint.y)
            return ObjectPosion::RIGHT;
         return ObjectPosion::LEFT;
      }
   }

//#################################################################################################
   ObjectPositionInfo cLaneAssignment::GetObjectPositionInfo(
                                                      std::vector<geometry_msgs::Point> vecPoint,
                                                      geometry_msgs::Point srcPoint)
   {
      ObjectPositionInfo oPointInfo;
      int nMinfMinDistance = std::numeric_limits<int>::max();
      int nSecondMin = std::numeric_limits<int>::max();
      geometry_msgs::Point oClosestPoint;
      geometry_msgs::Point oSecondClosestPoint;

      oPointInfo.fMinDistance = nMinfMinDistance;
      oPointInfo.ePosition = ObjectPosion::UNKNOWN;
      if (vecPoint.empty())
         return oPointInfo;

      for (auto target : vecPoint)
      {
         int diff = pow(pow(target.x - srcPoint.x,2) + pow(target.y - srcPoint.y,2),0.5);
         if (diff < nMinfMinDistance)
         {
            nMinfMinDistance = diff;
            oClosestPoint = target;
         }
         else if (diff < nSecondMin)
         {
            nSecondMin = diff;
            oSecondClosestPoint = target;
         }
      }

      oPointInfo.fMinDistance = nMinfMinDistance;
      oPointInfo.oClosestPoint = oClosestPoint;
      oPointInfo.ePosition = GetObjectPosition(srcPoint, oClosestPoint, oSecondClosestPoint);
      return oPointInfo;
   }

//#################################################################################################
   void cLaneAssignment::AssignLaneToObjects(outputBus_msgs::SensorDetectedObjectList oObjectData)
   {
      if(oObjectData.object_list.empty()) return;
 
      for (outputBus_msgs::SedSensorDetectedObject &currObject : oObjectData.object_list)
      {
         ObjectPositionInfo PointLeftInfo = GetObjectPositionInfo(
             m_mapLanePosition[LanePosition::EGO_LEFT].worldPoints, currObject.position);

         ObjectPositionInfo PointRightInfo = GetObjectPositionInfo(
             m_mapLanePosition[LanePosition::EGO_RIGHT].worldPoints, currObject.position);

         // Case when no lane data is comming
         if(ObjectPosion::UNKNOWN == PointLeftInfo.ePosition && ObjectPosion::UNKNOWN == PointRightInfo.ePosition)
         {
            currObject.classification = LanePosition::UNDEFINED;
            oObjectData.object_list.push_back(currObject);
            continue;
         }

         if (PointRightInfo.fMinDistance < PointLeftInfo.fMinDistance)
         {
            if(ObjectPosion::LEFT == PointRightInfo.ePosition)
            {
               currObject.lane_assignment = LaneAssignmnetPosition::LA_EGO_LANE;
               currObject.lane_id = m_mapLanePosition[LanePosition::EGO_LEFT].id;
            }
            else
            {
               ObjectPositionInfo oObjectInfo = AssignFromRightLanes(currObject.position);
               currObject.lane_assignment = oObjectInfo.laneAssignment;
               currObject.lane_id = oObjectInfo.laneId;
            }
         }
         else
         {
            if(ObjectPosion::RIGHT == PointLeftInfo.ePosition)
            {
               currObject.lane_assignment = LaneAssignmnetPosition::LA_EGO_LANE;
               currObject.lane_id = m_mapLanePosition[LanePosition::EGO_RIGHT].id;
            }
            else
            {
               ObjectPositionInfo oObjectInfo =  AssignFromLeftLanes(currObject.position);
               currObject.lane_assignment = oObjectInfo.laneAssignment;
               currObject.lane_id = oObjectInfo.laneId;
            }
         }

         //currObject.classification = currObject.lane_assignment;
      }

      m_oPublisher.publish(oObjectData);
   }
//#################################################################################################
   ObjectPositionInfo cLaneAssignment::AssignFromLeftLanes(geometry_msgs::Point srcPoint)
   {
      // Right Lane Set, assigned in order most left to least least lane 
      const LanePosition arrLeftLane[] = { LanePosition::EGO_LEFT_RIGHT,
                                           LanePosition::EGO_LEFT_LEFT,
                                           LanePosition::EGO_LEFT};

      ObjectPositionInfo pos;
      pos.ePosition = ObjectPosion::UNKNOWN;
      pos.fMinDistance = std::numeric_limits<double>::max();
      for (auto currLane : arrLeftLane)
      {
         ObjectPositionInfo posFromCurrLane  = GetObjectPositionInfo(
                                                   m_mapLanePosition[currLane].worldPoints,
                                                   srcPoint);
         pos = posFromCurrLane;
         pos.laneAssignment = LanePosToLaneAssignmnet(currLane);
         pos.laneId = m_mapLanePosition[currLane].id;
         if (pos.ePosition == ObjectPosion::LEFT)
            return pos;
      }
      return pos;
   }

//#################################################################################################
   ObjectPositionInfo cLaneAssignment::AssignFromRightLanes(geometry_msgs::Point srcPoint)
   {
      // Right Lane Set, assigned in order most right to least right lane 
      const LanePosition arrRightLane[] = { LanePosition::EGO_RIGHT_LEFT,
                                            LanePosition::EGO_RIGHT_RIGHT,
                                            LanePosition::EGO_RIGHT
                                          };

      ObjectPositionInfo pos;
      pos.fMinDistance = std::numeric_limits<double>::max();
      for (auto currLane : arrRightLane)
      {

         ObjectPositionInfo posFromCurrLane  = GetObjectPositionInfo(
                                                   m_mapLanePosition[currLane].worldPoints,
                                                   srcPoint);
         pos = posFromCurrLane;
         pos.laneAssignment = LanePosToLaneAssignmnet(currLane);
         pos.laneId = m_mapLanePosition[currLane].id;
         if(pos.ePosition == ObjectPosion::RIGHT)
            return pos;
      }
      return pos;
   }

//#################################################################################################
   void cLaneAssignment::PopulateLanePositionMap(inputBus_msgs::LaneMarkingArray oLaneData)
   {
      m_mapLanePosition.erase(m_mapLanePosition.begin(), m_mapLanePosition.end());

      for (auto lane : oLaneData.lanes)
      {
         switch(lane.position)
         {
            case LanePosition::UNDEFINED:
               m_mapLanePosition[LanePosition::UNDEFINED] = lane;
               break;
            case LanePosition::EGO_LEFT_RIGHT:
               m_mapLanePosition[LanePosition::EGO_LEFT_RIGHT] = lane;
               break;
            case LanePosition::EGO_LEFT:
               m_mapLanePosition[LanePosition::EGO_LEFT] = lane;
               break;
            case LanePosition::EGO_LEFT_LEFT:
               m_mapLanePosition[LanePosition::EGO_LEFT_LEFT] = lane;
               break;
            case LanePosition::EGO_RIGHT:
               m_mapLanePosition[LanePosition::EGO_RIGHT] = lane;
               break;
            case LanePosition::EGO_RIGHT_RIGHT:
               m_mapLanePosition[LanePosition::EGO_RIGHT_RIGHT] = lane;
               break;
            case LanePosition::EGO_RIGHT_LEFT:
               m_mapLanePosition[LanePosition::EGO_RIGHT_LEFT] = lane;
               break;
         }
      }
   }