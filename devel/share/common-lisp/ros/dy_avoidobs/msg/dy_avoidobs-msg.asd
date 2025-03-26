
(cl:in-package :asdf)

(defsystem "dy_avoidobs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "DataDisp" :depends-on ("_package_DataDisp"))
    (:file "_package_DataDisp" :depends-on ("_package"))
    (:file "Multilocaltrajs" :depends-on ("_package_Multilocaltrajs"))
    (:file "_package_Multilocaltrajs" :depends-on ("_package"))
    (:file "localtraj" :depends-on ("_package_localtraj"))
    (:file "_package_localtraj" :depends-on ("_package"))
  ))