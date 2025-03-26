;; Auto-generated. Do not edit!


(when (boundp 'dy_avoidobs::localtraj)
  (if (not (find-package "DY_AVOIDOBS"))
    (make-package "DY_AVOIDOBS"))
  (shadow 'localtraj (find-package "DY_AVOIDOBS")))
(unless (find-package "DY_AVOIDOBS::LOCALTRAJ")
  (make-package "DY_AVOIDOBS::LOCALTRAJ"))

(in-package "ROS")
;;//! \htmlinclude localtraj.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass dy_avoidobs::localtraj
  :super ros::object
  :slots (_car_id _traj_id _start_time _pos_pts ))

(defmethod dy_avoidobs::localtraj
  (:init
   (&key
    ((:car_id __car_id) 0)
    ((:traj_id __traj_id) 0)
    ((:start_time __start_time) (instance ros::time :init))
    ((:pos_pts __pos_pts) ())
    )
   (send-super :init)
   (setq _car_id (round __car_id))
   (setq _traj_id (round __traj_id))
   (setq _start_time __start_time)
   (setq _pos_pts __pos_pts)
   self)
  (:car_id
   (&optional __car_id)
   (if __car_id (setq _car_id __car_id)) _car_id)
  (:traj_id
   (&optional __traj_id)
   (if __traj_id (setq _traj_id __traj_id)) _traj_id)
  (:start_time
   (&optional __start_time)
   (if __start_time (setq _start_time __start_time)) _start_time)
  (:pos_pts
   (&rest __pos_pts)
   (if (keywordp (car __pos_pts))
       (send* _pos_pts __pos_pts)
     (progn
       (if __pos_pts (setq _pos_pts (car __pos_pts)))
       _pos_pts)))
  (:serialization-length
   ()
   (+
    ;; int32 _car_id
    4
    ;; int64 _traj_id
    8
    ;; time _start_time
    8
    ;; geometry_msgs/Point[] _pos_pts
    (apply #'+ (send-all _pos_pts :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _car_id
       (write-long _car_id s)
     ;; int64 _traj_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _traj_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _traj_id) (= (length (_traj_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_traj_id . bv) 0) 0) s)
              (write-long (ash (elt (_traj_id . bv) 1) -1) s))
             ((and (class _traj_id) (= (length (_traj_id . bv)) 1)) ;; big1
              (write-long (elt (_traj_id . bv) 0) s)
              (write-long (if (>= _traj_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _traj_id s)(write-long (if (>= _traj_id 0) 0 #xffffffff) s)))
     ;; time _start_time
       (write-long (send _start_time :sec) s) (write-long (send _start_time :nsec) s)
     ;; geometry_msgs/Point[] _pos_pts
     (write-long (length _pos_pts) s)
     (dolist (elem _pos_pts)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _car_id
     (setq _car_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int64 _traj_id
#+(or :alpha :irix6 :x86_64)
      (setf _traj_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _traj_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; time _start_time
     (send _start_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _start_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point[] _pos_pts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pos_pts (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _pos_pts)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dy_avoidobs::localtraj :md5sum-) "3dc77ae9fb11c09e1886c8d863ff6e72")
(setf (get dy_avoidobs::localtraj :datatype-) "dy_avoidobs/localtraj")
(setf (get dy_avoidobs::localtraj :definition-)
      "int32 car_id

int64 traj_id
time start_time


geometry_msgs/Point[] pos_pts




================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :dy_avoidobs/localtraj "3dc77ae9fb11c09e1886c8d863ff6e72")


