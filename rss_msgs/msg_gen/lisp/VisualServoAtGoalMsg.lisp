; Auto-generated. Do not edit!


(cl:in-package rss_msgs-msg)


;//! \htmlinclude VisualServoAtGoalMsg.msg.html

(cl:defclass <VisualServoAtGoalMsg> (roslisp-msg-protocol:ros-message)
  ((atGoal
    :reader atGoal
    :initarg :atGoal
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VisualServoAtGoalMsg (<VisualServoAtGoalMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualServoAtGoalMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualServoAtGoalMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rss_msgs-msg:<VisualServoAtGoalMsg> is deprecated: use rss_msgs-msg:VisualServoAtGoalMsg instead.")))

(cl:ensure-generic-function 'atGoal-val :lambda-list '(m))
(cl:defmethod atGoal-val ((m <VisualServoAtGoalMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rss_msgs-msg:atGoal-val is deprecated.  Use rss_msgs-msg:atGoal instead.")
  (atGoal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualServoAtGoalMsg>) ostream)
  "Serializes a message object of type '<VisualServoAtGoalMsg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'atGoal) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualServoAtGoalMsg>) istream)
  "Deserializes a message object of type '<VisualServoAtGoalMsg>"
    (cl:setf (cl:slot-value msg 'atGoal) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualServoAtGoalMsg>)))
  "Returns string type for a message object of type '<VisualServoAtGoalMsg>"
  "rss_msgs/VisualServoAtGoalMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualServoAtGoalMsg)))
  "Returns string type for a message object of type 'VisualServoAtGoalMsg"
  "rss_msgs/VisualServoAtGoalMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualServoAtGoalMsg>)))
  "Returns md5sum for a message object of type '<VisualServoAtGoalMsg>"
  "8871aefd0a49329f07f75943cf152ccd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualServoAtGoalMsg)))
  "Returns md5sum for a message object of type 'VisualServoAtGoalMsg"
  "8871aefd0a49329f07f75943cf152ccd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualServoAtGoalMsg>)))
  "Returns full string definition for message of type '<VisualServoAtGoalMsg>"
  (cl:format cl:nil "bool atGoal~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualServoAtGoalMsg)))
  "Returns full string definition for message of type 'VisualServoAtGoalMsg"
  (cl:format cl:nil "bool atGoal~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualServoAtGoalMsg>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualServoAtGoalMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualServoAtGoalMsg
    (cl:cons ':atGoal (atGoal msg))
))
