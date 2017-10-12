; Auto-generated. Do not edit!


(cl:in-package rebvo-msg)


;//! \htmlinclude Keyline.msg.html

(cl:defclass <Keyline> (roslisp-msg-protocol:ros-message)
  ((KlGrad
    :reader KlGrad
    :initarg :KlGrad
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (KlImgPos
    :reader KlImgPos
    :initarg :KlImgPos
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (invDepth
    :reader invDepth
    :initarg :invDepth
    :type cl:float
    :initform 0.0)
   (invDepthS
    :reader invDepthS
    :initarg :invDepthS
    :type cl:float
    :initform 0.0)
   (KlFocPos
    :reader KlFocPos
    :initarg :KlFocPos
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (KlMatchID
    :reader KlMatchID
    :initarg :KlMatchID
    :type cl:integer
    :initform 0)
   (ConsMatch
    :reader ConsMatch
    :initarg :ConsMatch
    :type cl:integer
    :initform 0)
   (KlPrevMatchID
    :reader KlPrevMatchID
    :initarg :KlPrevMatchID
    :type cl:fixnum
    :initform 0)
   (KlNextMatchID
    :reader KlNextMatchID
    :initarg :KlNextMatchID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Keyline (<Keyline>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Keyline>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Keyline)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rebvo-msg:<Keyline> is deprecated: use rebvo-msg:Keyline instead.")))

(cl:ensure-generic-function 'KlGrad-val :lambda-list '(m))
(cl:defmethod KlGrad-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlGrad-val is deprecated.  Use rebvo-msg:KlGrad instead.")
  (KlGrad m))

(cl:ensure-generic-function 'KlImgPos-val :lambda-list '(m))
(cl:defmethod KlImgPos-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlImgPos-val is deprecated.  Use rebvo-msg:KlImgPos instead.")
  (KlImgPos m))

(cl:ensure-generic-function 'invDepth-val :lambda-list '(m))
(cl:defmethod invDepth-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:invDepth-val is deprecated.  Use rebvo-msg:invDepth instead.")
  (invDepth m))

(cl:ensure-generic-function 'invDepthS-val :lambda-list '(m))
(cl:defmethod invDepthS-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:invDepthS-val is deprecated.  Use rebvo-msg:invDepthS instead.")
  (invDepthS m))

(cl:ensure-generic-function 'KlFocPos-val :lambda-list '(m))
(cl:defmethod KlFocPos-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlFocPos-val is deprecated.  Use rebvo-msg:KlFocPos instead.")
  (KlFocPos m))

(cl:ensure-generic-function 'KlMatchID-val :lambda-list '(m))
(cl:defmethod KlMatchID-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlMatchID-val is deprecated.  Use rebvo-msg:KlMatchID instead.")
  (KlMatchID m))

(cl:ensure-generic-function 'ConsMatch-val :lambda-list '(m))
(cl:defmethod ConsMatch-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:ConsMatch-val is deprecated.  Use rebvo-msg:ConsMatch instead.")
  (ConsMatch m))

(cl:ensure-generic-function 'KlPrevMatchID-val :lambda-list '(m))
(cl:defmethod KlPrevMatchID-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlPrevMatchID-val is deprecated.  Use rebvo-msg:KlPrevMatchID instead.")
  (KlPrevMatchID m))

(cl:ensure-generic-function 'KlNextMatchID-val :lambda-list '(m))
(cl:defmethod KlNextMatchID-val ((m <Keyline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rebvo-msg:KlNextMatchID-val is deprecated.  Use rebvo-msg:KlNextMatchID instead.")
  (KlNextMatchID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Keyline>) ostream)
  "Serializes a message object of type '<Keyline>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'KlGrad))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'KlImgPos))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'invDepth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'invDepthS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'KlFocPos))
  (cl:let* ((signed (cl:slot-value msg 'KlMatchID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ConsMatch)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'KlPrevMatchID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'KlNextMatchID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Keyline>) istream)
  "Deserializes a message object of type '<Keyline>"
  (cl:setf (cl:slot-value msg 'KlGrad) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'KlGrad)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'KlImgPos) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'KlImgPos)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'invDepth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'invDepthS) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'KlFocPos) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'KlFocPos)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'KlMatchID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ConsMatch) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'KlPrevMatchID) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'KlNextMatchID) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Keyline>)))
  "Returns string type for a message object of type '<Keyline>"
  "rebvo/Keyline")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Keyline)))
  "Returns string type for a message object of type 'Keyline"
  "rebvo/Keyline")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Keyline>)))
  "Returns md5sum for a message object of type '<Keyline>"
  "1ae0c73cd497f30da6aa8e3479b24957")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Keyline)))
  "Returns md5sum for a message object of type 'Keyline"
  "1ae0c73cd497f30da6aa8e3479b24957")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Keyline>)))
  "Returns full string definition for message of type '<Keyline>"
  (cl:format cl:nil "float32[2] KlGrad		#KeyLine's gradient vector [x,y]~%~%float32[2] KlImgPos		#KeyLine's image position [x,y]~%~%float64 invDepth		#Estimated Inverse Depth~%float64 invDepthS		#Estimated Inverse Depth Uncertainty~%~%float32[2] KlFocPos		#KL position in focal point centred coordinates (plane at focal length zf) [x,y]~%~%int32 KlMatchID			#Id of the matching keyline~%~%int32 ConsMatch			#number of consecutive matches~%~%int16 KlPrevMatchID		#Id of previous consecutive KeyLine~%int16 KlNextMatchID		#Id of next consecutive KeyLine~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Keyline)))
  "Returns full string definition for message of type 'Keyline"
  (cl:format cl:nil "float32[2] KlGrad		#KeyLine's gradient vector [x,y]~%~%float32[2] KlImgPos		#KeyLine's image position [x,y]~%~%float64 invDepth		#Estimated Inverse Depth~%float64 invDepthS		#Estimated Inverse Depth Uncertainty~%~%float32[2] KlFocPos		#KL position in focal point centred coordinates (plane at focal length zf) [x,y]~%~%int32 KlMatchID			#Id of the matching keyline~%~%int32 ConsMatch			#number of consecutive matches~%~%int16 KlPrevMatchID		#Id of previous consecutive KeyLine~%int16 KlNextMatchID		#Id of next consecutive KeyLine~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Keyline>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'KlGrad) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'KlImgPos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     8
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'KlFocPos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Keyline>))
  "Converts a ROS message object to a list"
  (cl:list 'Keyline
    (cl:cons ':KlGrad (KlGrad msg))
    (cl:cons ':KlImgPos (KlImgPos msg))
    (cl:cons ':invDepth (invDepth msg))
    (cl:cons ':invDepthS (invDepthS msg))
    (cl:cons ':KlFocPos (KlFocPos msg))
    (cl:cons ':KlMatchID (KlMatchID msg))
    (cl:cons ':ConsMatch (ConsMatch msg))
    (cl:cons ':KlPrevMatchID (KlPrevMatchID msg))
    (cl:cons ':KlNextMatchID (KlNextMatchID msg))
))
