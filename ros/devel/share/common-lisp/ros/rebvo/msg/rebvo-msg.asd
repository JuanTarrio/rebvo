
(cl:in-package :asdf)

(defsystem "rebvo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Keyline" :depends-on ("_package_Keyline"))
    (:file "_package_Keyline" :depends-on ("_package"))
    (:file "EdgeMap" :depends-on ("_package_EdgeMap"))
    (:file "_package_EdgeMap" :depends-on ("_package"))
  ))