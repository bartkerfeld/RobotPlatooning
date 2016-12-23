; Auto-generated. Do not edit!


(cl:in-package nav_common-msg)


;//! \htmlinclude movement_request.msg.html

(cl:defclass <movement_request> (roslisp-msg-protocol:ros-message)
  ((category
    :reader category
    :initarg :category
    :type cl:string
    :initform "")
   (subclass
    :reader subclass
    :initarg :subclass
    :type cl:string
    :initform ""))
)

(cl:defclass movement_request (<movement_request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <movement_request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'movement_request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nav_common-msg:<movement_request> is deprecated: use nav_common-msg:movement_request instead.")))

(cl:ensure-generic-function 'category-val :lambda-list '(m))
(cl:defmethod category-val ((m <movement_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav_common-msg:category-val is deprecated.  Use nav_common-msg:category instead.")
  (category m))

(cl:ensure-generic-function 'subclass-val :lambda-list '(m))
(cl:defmethod subclass-val ((m <movement_request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nav_common-msg:subclass-val is deprecated.  Use nav_common-msg:subclass instead.")
  (subclass m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <movement_request>) ostream)
  "Serializes a message object of type '<movement_request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'category))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'category))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'subclass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'subclass))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <movement_request>) istream)
  "Deserializes a message object of type '<movement_request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'category) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'category) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'subclass) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'subclass) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<movement_request>)))
  "Returns string type for a message object of type '<movement_request>"
  "nav_common/movement_request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'movement_request)))
  "Returns string type for a message object of type 'movement_request"
  "nav_common/movement_request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<movement_request>)))
  "Returns md5sum for a message object of type '<movement_request>"
  "f034dc06870086475e1cf8ef445bd4da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'movement_request)))
  "Returns md5sum for a message object of type 'movement_request"
  "f034dc06870086475e1cf8ef445bd4da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<movement_request>)))
  "Returns full string definition for message of type '<movement_request>"
  (cl:format cl:nil "string category~%string subclass~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'movement_request)))
  "Returns full string definition for message of type 'movement_request"
  (cl:format cl:nil "string category~%string subclass~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <movement_request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'category))
     4 (cl:length (cl:slot-value msg 'subclass))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <movement_request>))
  "Converts a ROS message object to a list"
  (cl:list 'movement_request
    (cl:cons ':category (category msg))
    (cl:cons ':subclass (subclass msg))
))
