;; Auto-generated. Do not edit!


(when (boundp 'path_follower::GetPath)
  (if (not (find-package "PATH_FOLLOWER"))
    (make-package "PATH_FOLLOWER"))
  (shadow 'GetPath (find-package "PATH_FOLLOWER")))
(unless (find-package "PATH_FOLLOWER::GETPATH")
  (make-package "PATH_FOLLOWER::GETPATH"))
(unless (find-package "PATH_FOLLOWER::GETPATHREQUEST")
  (make-package "PATH_FOLLOWER::GETPATHREQUEST"))
(unless (find-package "PATH_FOLLOWER::GETPATHRESPONSE")
  (make-package "PATH_FOLLOWER::GETPATHRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass path_follower::GetPathRequest
  :super ros::object
  :slots ())

(defmethod path_follower::GetPathRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass path_follower::GetPathResponse
  :super ros::object
  :slots (_path ))

(defmethod path_follower::GetPathResponse
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(defclass path_follower::GetPath
  :super ros::object
  :slots ())

(setf (get path_follower::GetPath :md5sum-) "58d6f138c7de7ef47c75d4b7e5df5472")
(setf (get path_follower::GetPath :datatype-) "path_follower/GetPath")
(setf (get path_follower::GetPath :request) path_follower::GetPathRequest)
(setf (get path_follower::GetPath :response) path_follower::GetPathResponse)

(defmethod path_follower::GetPathRequest
  (:response () (instance path_follower::GetPathResponse :init)))

(setf (get path_follower::GetPathRequest :md5sum-) "58d6f138c7de7ef47c75d4b7e5df5472")
(setf (get path_follower::GetPathRequest :datatype-) "path_follower/GetPathRequest")
(setf (get path_follower::GetPathRequest :definition-)
      "---
nav_msgs/Path path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get path_follower::GetPathResponse :md5sum-) "58d6f138c7de7ef47c75d4b7e5df5472")
(setf (get path_follower::GetPathResponse :datatype-) "path_follower/GetPathResponse")
(setf (get path_follower::GetPathResponse :definition-)
      "---
nav_msgs/Path path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :path_follower/GetPath "58d6f138c7de7ef47c75d4b7e5df5472")


