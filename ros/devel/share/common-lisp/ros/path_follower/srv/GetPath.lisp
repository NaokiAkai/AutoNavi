; Auto-generated. Do not edit!


(cl:in-package path_follower-srv)


;//! \htmlinclude GetPath-request.msg.html

(cl:defclass <GetPath-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetPath-request (<GetPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-srv:<GetPath-request> is deprecated: use path_follower-srv:GetPath-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-request>) ostream)
  "Serializes a message object of type '<GetPath-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-request>) istream)
  "Deserializes a message object of type '<GetPath-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-request>)))
  "Returns string type for a service object of type '<GetPath-request>"
  "path_follower/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-request)))
  "Returns string type for a service object of type 'GetPath-request"
  "path_follower/GetPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-request>)))
  "Returns md5sum for a message object of type '<GetPath-request>"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-request)))
  "Returns md5sum for a message object of type 'GetPath-request"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-request>)))
  "Returns full string definition for message of type '<GetPath-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-request)))
  "Returns full string definition for message of type 'GetPath-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-request
))
;//! \htmlinclude GetPath-response.msg.html

(cl:defclass <GetPath-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass GetPath-response (<GetPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name path_follower-srv:<GetPath-response> is deprecated: use path_follower-srv:GetPath-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <GetPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader path_follower-srv:path-val is deprecated.  Use path_follower-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPath-response>) ostream)
  "Serializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPath-response>) istream)
  "Deserializes a message object of type '<GetPath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPath-response>)))
  "Returns string type for a service object of type '<GetPath-response>"
  "path_follower/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath-response)))
  "Returns string type for a service object of type 'GetPath-response"
  "path_follower/GetPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPath-response>)))
  "Returns md5sum for a message object of type '<GetPath-response>"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPath-response)))
  "Returns md5sum for a message object of type 'GetPath-response"
  "58d6f138c7de7ef47c75d4b7e5df5472")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPath-response>)))
  "Returns full string definition for message of type '<GetPath-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPath-response)))
  "Returns full string definition for message of type 'GetPath-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPath-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPath)))
  'GetPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPath)))
  'GetPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPath)))
  "Returns string type for a service object of type '<GetPath>"
  "path_follower/GetPath")