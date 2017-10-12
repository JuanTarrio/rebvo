; Auto-generated. Do not edit!


(cl:in-package rebvo-msg)


;//! \htmlinclude EdgeMap.msg.html

(cl:defclass <EdgeMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (Keylines
    :reader Keylines
    :initarg :Keylines
    :type (cl:vector rebvo-msg:Keyline)
   :initform (cl:make-array 0 :element-type 'rebvo-msg:Keyline :initial-element (cl:make-instance 'rebvo-msg:Keyline))))
)

(cl:defclass EdgeMap (<EdgeMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EdgeMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EdgeMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rebvo-msg:<EdgeMap> is deprecated: use rebvo-msg:EdgeMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EdgeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:header-val is deprecated.  Use rebvo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'Keylines-val :lambda-list '(m))
(cl:defmethod Keylines-val ((m <EdgeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:Keylines-val is deprecated.  Use rebvo-msg:Keylines instead.")
  (Keylines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EdgeMap>) ostream)
  "Serializes a message object of type '<EdgeMap>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Keylines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'Keylines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EdgeMap>) istream)
  "Deserializes a message object of type '<EdgeMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Keylines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Keylines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'rebvo-msg:Keyline))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EdgeMap>)))
  "Returns string type for a message object of type '<EdgeMap>"
  "rebvo/EdgeMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EdgeMap)))
  "Returns string type for a message object of type 'EdgeMap"
  "rebvo/EdgeMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EdgeMap>)))
  "Returns md5sum for a message object of type '<EdgeMap>"
  "885ab556237b8cc71cfc39f60b72964e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EdgeMap)))
  "Returns md5sum for a message object of type 'EdgeMap"
  "885ab556237b8cc71cfc39f60b72964e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EdgeMap>)))
  "Returns full string definition for message of type '<EdgeMap>"
  (cl:format cl:nil "Header header		#KeyLine header~%~%Keyline[] Keylines	#EdgeMap (KeyLines detected)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rebvo/Keyline~%float32[2] KlGrad		#KeyLine's gradient vector [x,y]~%~%float32[2] KlImgPos		#KeyLine's image position [x,y]~%~%float64 invDepth		#Estimated Inverse Depth~%float64 invDepthS		#Estimated Inverse Depth Uncertainty~%~%float32[2] KlFocPos		#KL position in focal point centred coordinates (plane at focal length zf) [x,y]~%~%int32 KlMatchID			#Id of the matching keyline~%~%int32 ConsMatch			#number of consecutive matches~%~%int16 KlPrevMatchID		#Id of previous consecutive KeyLine~%int16 KlNextMatchID		#Id of next consecutive KeyLine~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EdgeMap)))
  "Returns full string definition for message of type 'EdgeMap"
  (cl:format cl:nil "Header header		#KeyLine header~%~%Keyline[] Keylines	#EdgeMap (KeyLines detected)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: rebvo/Keyline~%float32[2] KlGrad		#KeyLine's gradient vector [x,y]~%~%float32[2] KlImgPos		#KeyLine's image position [x,y]~%~%float64 invDepth		#Estimated Inverse Depth~%float64 invDepthS		#Estimated Inverse Depth Uncertainty~%~%float32[2] KlFocPos		#KL position in focal point centred coordinates (plane at focal length zf) [x,y]~%~%int32 KlMatchID			#Id of the matching keyline~%~%int32 ConsMatch			#number of consecutive matches~%~%int16 KlPrevMatchID		#Id of previous consecutive KeyLine~%int16 KlNextMatchID		#Id of next consecutive KeyLine~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EdgeMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Keylines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EdgeMap>))
  "Converts a ROS message object to a list"
  (cl:list 'EdgeMap
    (cl:cons ':header (header msg))
    (cl:cons ':Keylines (Keylines msg))
))
