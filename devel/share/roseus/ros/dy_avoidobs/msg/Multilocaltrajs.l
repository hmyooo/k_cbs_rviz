;; Auto-generated. Do not edit!


(when (boundp 'dy_avoidobs::Multilocaltrajs)
  (if (not (find-package "DY_AVOIDOBS"))
    (make-package "DY_AVOIDOBS"))
  (shadow 'Multilocaltrajs (find-package "DY_AVOIDOBS")))
(unless (find-package "DY_AVOIDOBS::MULTILOCALTRAJS")
  (make-package "DY_AVOIDOBS::MULTILOCALTRAJS"))

(in-package "ROS")
;;//! \htmlinclude Multilocaltrajs.msg.html


(defclass dy_avoidobs::Multilocaltrajs
  :super ros::object
  :slots (_car_id_from _traj ))

(defmethod dy_avoidobs::Multilocaltrajs
  (:init
   (&key
    ((:car_id_from __car_id_from) 0)
    ((:traj __traj) ())
    )
   (send-super :init)
   (setq _car_id_from (round __car_id_from))
   (setq _traj __traj)
   self)
  (:car_id_from
   (&optional __car_id_from)
   (if __car_id_from (setq _car_id_from __car_id_from)) _car_id_from)
  (:traj
   (&rest __traj)
   (if (keywordp (car __traj))
       (send* _traj __traj)
     (progn
       (if __traj (setq _traj (car __traj)))
       _traj)))
  (:serialization-length
   ()
   (+
    ;; int32 _car_id_from
    4
    ;; dy_avoidobs/localtraj[] _traj
    (apply #'+ (send-all _traj :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _car_id_from
       (write-long _car_id_from s)
     ;; dy_avoidobs/localtraj[] _traj
     (write-long (length _traj) s)
     (dolist (elem _traj)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _car_id_from
     (setq _car_id_from (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; dy_avoidobs/localtraj[] _traj
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _traj (let (r) (dotimes (i n) (push (instance dy_avoidobs::localtraj :init) r)) r))
     (dolist (elem- _traj)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get dy_avoidobs::Multilocaltrajs :md5sum-) "3c3a990cdcf595dfcffe37275da7c4d3")
(setf (get dy_avoidobs::Multilocaltrajs :datatype-) "dy_avoidobs/Multilocaltrajs")
(setf (get dy_avoidobs::Multilocaltrajs :definition-)
      "int32 car_id_from

localtraj[] traj


================================================================================
MSG: dy_avoidobs/localtraj
int32 car_id

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



(provide :dy_avoidobs/Multilocaltrajs "3c3a990cdcf595dfcffe37275da7c4d3")


