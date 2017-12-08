
(cl:in-package :asdf)

(defsystem "path_follower-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "GetPath" :depends-on ("_package_GetPath"))
    (:file "_package_GetPath" :depends-on ("_package"))
  ))