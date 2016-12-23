
(cl:in-package :asdf)

(defsystem "nav_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "movement_request" :depends-on ("_package_movement_request"))
    (:file "_package_movement_request" :depends-on ("_package"))
  ))