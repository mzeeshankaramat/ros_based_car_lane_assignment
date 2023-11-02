/*
 * @filename    cLaneAssignment.h
 * @author      Zeeshan Karamat
 */

#ifndef PROJECT_LANEASSIGNMENT_H
#define PROJECT_LANEASSIGNMENT_H

#include <ros/ros.h>


#include <ros/ros.h>
#include <inputBus_msgs/LaneMarkingArray.h>
#include <outputBus_msgs/SensorDetectedObjectList.h>
#include <inputBus_msgs/SensorDetectedObjectList.h>
#include <mutex>


//#####Defines#############################################################
#define DEFAULT_OBJECT_LIST_TOPIC_NAME "/input_objects"
#define DEFAULT_LANE_TOPIC_NAME "/input_Lanes"
#define DEFAULT_OUTPUT_TOPIC_NAME "/LaneAssignment/Objects"
#define DEFAULT_DETAIL_TOPIC_NAME "/detail_info"



#define OBJECT_INPUT_TOPIC_PARAM "input_topic_object"
#define LANE_INPUT_TOPIC_PARAM "input_topic_lane"
#define OBJECT_OUTPUT_TOPIC_PARAM "output_topic"

#define QUEUE_SIZE 10

enum LanePosition
{
   UNDEFINED = inputBus_msgs::LaneMarking::POSITION_UNDEFINED,
   EGO_LEFT_RIGHT = inputBus_msgs::LaneMarking::POSITION_ADJACENT_LEFT_RIGHT,
   EGO_LEFT_LEFT = inputBus_msgs::LaneMarking::POSITION_ADJACENT_LEFT,
   EGO_LEFT = inputBus_msgs::LaneMarking::POSITION_EGO_LEFT,
   EGO_RIGHT = inputBus_msgs::LaneMarking::POSITION_EGO_RIGHT,
   EGO_RIGHT_RIGHT = inputBus_msgs::LaneMarking::POSITION_ADJACENT_RIGHT,
   EGO_RIGHT_LEFT = inputBus_msgs::LaneMarking::POSITION_ADJACENT_RIGHT_LEFT
};

enum LaneAssignmnetPosition
{
   LA_UNKNOWN = outputBus_msgs::SensorDetectedObject::LA_UNKNOWN,
   LA_LEFT_LEFT_LANE = outputBus_msgs::SensorDetectedObject::LA_LEFT_LEFT_LANE,
   LA_LEFT_LANE = outputBus_msgs::SensorDetectedObject::LA_LEFT_LANE,
   LA_EGO_LANE = outputBus_msgs::SensorDetectedObject::LA_EGO_LANE,
   LA_RIGHT_LANE = outputBus_msgs::SensorDetectedObject::LA_RIGHT_LANE,
   LA_RIGHT_RIGHT_LANE = outputBus_msgs::SensorDetectedObject::LA_RIGHT_RIGHT_LANE,

   LA_PAVEMENT_LEFT_LANE = outputBus_msgs::SensorDetectedObject::LA_PAVEMENT_LEFT_LANE,
   LA_BYCYCLE_LEFT_LANE = outputBus_msgs::SensorDetectedObject::LA_BYCYCLE_LEFT_LANE,
   LA_PAVEMENT_RIGHT_LANE = outputBus_msgs::SensorDetectedObject::LA_PAVEMENT_RIGHT_LANE,
   LA_BYCYCLE_RIGHT_LANE = outputBus_msgs::SensorDetectedObject::LA_BYCYCLE_RIGHT_LANE,
};

//Enum for ObjectPosion, Object Location relative to lane
enum ObjectPosion
{
   LEFT  = 0,
   RIGHT = 1,
   UNKNOWN = 2
};

//#################################### Structs #################################################
struct ObjectPositionInfo
{
   /**
    * min distance of object from lane points 
    */ 
   double fMinDistance;

   /**
    * closest point to object from lane point list
    */ 
   geometry_msgs::Point oClosestPoint;

   /**
    * object position i.e left of lane or right of lane
    */ 
   ObjectPosion ePosition;

   /**
    * lane on which object lies
    */ 
   int laneAssignment;
   int laneId;
};

class cLaneAssignment
{

public:
/**
 * @brief constractor 
 * @param pubicHandle public node handle
 * @param privateHandle private node handle
 */
   cLaneAssignment(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle);


private:
   

   void LaneSubscriberCallback(const boost::shared_ptr<inputBus_msgs::LaneMarkingArray const>& pLaneMsg);
   void ObjectSubscriberCallback(
                     const boost::shared_ptr<inputBus_msgs::SensorDetectedObjectList const>& pObjMsg);


   /**
    * @brief Add lanes in map, key Lane Position  
    * @param oLaneData LaneMarkingArray lane data
    */
   void PopulateLanePositionMap(inputBus_msgs::LaneMarkingArray oLaneData);

   /**
    * @brief Assign Lanes to Objects  
    * @param oObjectData SensorDetectedObjectList object data
    */
   void AssignLaneToObjects(outputBus_msgs::SensorDetectedObjectList oObjectData);

   /**
    * @brief Assign lane from left lanes only
    * @param srcPoint source object position
    * @return  ObjectPositionInfo info of source object
    */
   ObjectPositionInfo AssignFromLeftLanes(geometry_msgs::Point srcPoint);
   
   /**
    * @brief Assign lane from right lanes only
    * @param srcPoint source object position
    * @return  ObjectPositionInfo info of source object
    */
   ObjectPositionInfo AssignFromRightLanes(geometry_msgs::Point srcPoint);


   /**
    * @brief Get PositionInfo of object relative to lane
    * @param vecPoint points of lane
    * @param srcPoint source object position
    * @return  ObjectPositionInfo info of source object
    */
   ObjectPositionInfo GetObjectPositionInfo(
                                 std::vector<geometry_msgs::Point> vecPoint,
                                 geometry_msgs::Point srcPoint);

    /**
    * @brief Get PositionInfo of object relative to lane
    * @param firstPoint first point of lane
    * @param secondPoint adjecent point of same lane
    * @param srcPoint source object position
    * @return  ObjectPosition 
    */                    
   ObjectPosion GetObjectPosition(geometry_msgs::Point srcPoint,
                                  geometry_msgs::Point firstPoint,
                                  geometry_msgs::Point secondPoint);



   ros::NodeHandle m_oPublicHandle;
   ros::NodeHandle m_oPrivateHandle;
   ros::Subscriber m_oLaneSubscriber;
   ros::Subscriber m_oObjectSubscriber;

   ros::Publisher m_oDetailInfoPublisher;
   ros::Publisher m_oPublisher;
   std::mutex m_lock;

   /**
    * @brief store lane data
    * @key LanePosition enum
    * @Value LaneMarking Data   
    */
   std::map<LanePosition, inputBus_msgs::LaneMarking> m_mapLanePosition;

   
};

#endif //PROJECT_LANEASSIGNMENT_H