cmake_minimum_required(VERSION 3.0.2)
project(interfaces_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            tf
            roscpp
            std_msgs
            geometry_msgs
            sensor_msgs
            rospy
            message_generation
        )

add_message_files(FILES 
  AutoComponentDebug.msg
  AutoDriveModeRequest.msg
  AutoNavigation.msg
  AutoRemoteControl.msg
  AutoUGVEStopRequest.msg
  AutoUGVShutdownRequest.msg
  ChassisCommand.msg
  ImageSize.msg
  NavigationDetail.msg
  Ouster128.msg
  PathTracker.msg
  SensorDataImage.msg
  StructSensorLidarRTheta.msg
  SensorDataLidarRTheta.msg
  ArraySensorDataLidarRTheta.msg
  ZoneChar.msg
  NavigationSystemStatus.msg
  NavigationStatus.msg
  GpsStatus.msg
  VehicleResponse.msg
  StructSpeedInfomation.msg
  StructBatteryStatus.msg
  StructEngineStatus.msg
  StructMileageInfomation.msg
  SatelliteStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        message_runtime
        roscpp
        tf
        sensor_msgs
        geometry_msgs
        std_msgs	
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)