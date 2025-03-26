; Auto-generated. Do not edit!


(cl:in-package dy_avoidobs-msg)


;//! \htmlinclude Multilocaltrajs.msg.html

(cl:defclass <Multilocaltrajs> (roslisp-msg-protocol:ros-message)
  ((car_id_from
    :reader car_id_from
    :initarg :car_id_from
    :type cl:integer
    :initform 0)
   (traj
    :reader traj
    :initarg :traj
    :type (cl:vector dy_avoidobs-msg:localtraj)
   :initform (cl:make-array 0 :element-type 'dy_avoidobs-msg:localtraj :initial-element (cl:make-instance 'dy_avoidobs-msg:localtraj))))
)

(cl:defclass Multilocaltrajs (<Multilocaltrajs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Multilocaltrajs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Multilocaltrajs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dy_avoidobs-msg:<Multilocaltrajs> is deprecated: use dy_avoidobs-msg:Multilocaltrajs instead.")))

(cl:ensure-generic-function 'car_id_from-val :lambda-list '(m))
(cl:defmethod car_id_from-val ((m <Multilocaltrajs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dy_avoidobs-msg:car_id_from-val is deprecated.  Use dy_avoidobs-msg:car_id_from instead.")
  (car_id_from m))

(cl:ensure-generic-function 'traj-val :lambda-list '(m))
(cl:defmethod traj-val ((m <Multilocaltrajs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dy_avoidobs-msg:traj-val is deprecated.  Use dy_avoidobs-msg:traj instead.")
  (traj m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Multilocaltrajs>) ostream)
  "Serializes a message object of type '<Multilocaltrajs>"
  (cl:let* ((signed (cl:slot-value msg 'car_id_from)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'traj))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'traj))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Multilocaltrajs>) istream)
  "Deserializes a message object of type '<Multilocaltrajs>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_id_from) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'traj) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'traj)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'dy_avoidobs-msg:localtraj))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Multilocaltrajs>)))
  "Returns string type for a message object of type '<Multilocaltrajs>"
  "dy_avoidobs/Multilocaltrajs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Multilocaltrajs)))
  "Returns string type for a message object of type 'Multilocaltrajs"
  "dy_avoidobs/Multilocaltrajs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Multilocaltrajs>)))
  "Returns md5sum for a message object of type '<Multilocaltrajs>"
  "3c3a990cdcf595dfcffe37275da7c4d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Multilocaltrajs)))
  "Returns md5sum for a message object of type 'Multilocaltrajs"
  "3c3a990cdcf595dfcffe37275da7c4d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Multilocaltrajs>)))
  "Returns full string definition for message of type '<Multilocaltrajs>"
  (cl:format cl:nil "int32 car_id_from~%~%localtraj[] traj~%~%~%================================================================================~%MSG: dy_avoidobs/localtraj~%int32 car_id~%~%int64 traj_id~%time start_time~%~%~%geometry_msgs/Point[] pos_pts~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Multilocaltrajs)))
  "Returns full string definition for message of type 'Multilocaltrajs"
  (cl:format cl:nil "int32 car_id_from~%~%localtraj[] traj~%~%~%================================================================================~%MSG: dy_avoidobs/localtraj~%int32 car_id~%~%int64 traj_id~%time start_time~%~%~%geometry_msgs/Point[] pos_pts~%~%~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Multilocaltrajs>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'traj) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Multilocaltrajs>))
  "Converts a ROS message object to a list"
  (cl:list 'Multilocaltrajs
    (cl:cons ':car_id_from (car_id_from msg))
    (cl:cons ':traj (traj msg))
))
