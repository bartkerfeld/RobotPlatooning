;; Auto-generated. Do not edit!


(when (boundp 'nav_common::movement_request)
  (if (not (find-package "NAV_COMMON"))
    (make-package "NAV_COMMON"))
  (shadow 'movement_request (find-package "NAV_COMMON")))
(unless (find-package "NAV_COMMON::MOVEMENT_REQUEST")
  (make-package "NAV_COMMON::MOVEMENT_REQUEST"))

(in-package "ROS")
;;//! \htmlinclude movement_request.msg.html


(defclass nav_common::movement_request
  :super ros::object
  :slots (_category _subclass ))

(defmethod nav_common::movement_request
  (:init
   (&key
    ((:category __category) "")
    ((:subclass __subclass) "")
    )
   (send-super :init)
   (setq _category (string __category))
   (setq _subclass (string __subclass))
   self)
  (:category
   (&optional __category)
   (if __category (setq _category __category)) _category)
  (:subclass
   (&optional __subclass)
   (if __subclass (setq _subclass __subclass)) _subclass)
  (:serialization-length
   ()
   (+
    ;; string _category
    4 (length _category)
    ;; string _subclass
    4 (length _subclass)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _category
       (write-long (length _category) s) (princ _category s)
     ;; string _subclass
       (write-long (length _subclass) s) (princ _subclass s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _category
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _category (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _subclass
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _subclass (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get nav_common::movement_request :md5sum-) "f034dc06870086475e1cf8ef445bd4da")
(setf (get nav_common::movement_request :datatype-) "nav_common/movement_request")
(setf (get nav_common::movement_request :definition-)
      "string category
string subclass

")



(provide :nav_common/movement_request "f034dc06870086475e1cf8ef445bd4da")


