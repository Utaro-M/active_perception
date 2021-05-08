#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")
(require "../../../rm_debris/rm_board/euslisp/functions.l")
(require "../../../rm_debris/rm_board/euslisp/client/make_plane_for_opt.l ")
(ros::roseus "display_grasp_candidates_server")
(ros::wait-for-service "display_grasp_candidates")

(setq candidates-coords-list (list))
;; put_points_on_image.pyへcandidatesを送る hands-intervalは両手の間隔
(defun send-grasp-candidates(&key (hands-interval (list  100 300 500)))
  (setq req (instance rm_debris_vision::GraspCandidatesRequest :init))
  (let*
      ((lis (list))
       (points-l-list (list))
       (points-r-list (list))
       (coords-tmp )
       (l-pos-wrt #f(0 0 0))
       (r-pos-wrt #f(0 0 0))
       (l-pos #f(0 0 0))
       (r-pos #f(0 0 0))
       (frame_id (instance  std_msgs::String :init)))
    (setq candidates-coords-list (list))
    (dotimes (i (length hands-interval))
      (setq coords-tmp (auto-set-reaching-targets-each-hand :pos-shift (elt hands-interval i)))
      ;; (setq candidates-coords-list (append (list coords-tmp) candidates-coords-list))
      (setq candidates-coords-list (list-insert coords-tmp 100 candidates-coords-list))
      ;; (format t "coords-tmp = ~A~%" coords-tmp)
      ;; (setq l-pos (send (send (elt coords-tmp 0) :copy-worldcoords) :worldpos))
      ;; (setq r-pos (send (send (elt coords-tmp 1) :copy-worldcoords) :worldpos))

      (setq l-pos-wrt (send (send (elt coords-tmp 0) :copy-worldcoords) :worldpos))
      (setq r-pos-wrt (send (send (elt coords-tmp 1) :copy-worldcoords) :worldpos))
      (setq waist-coords (send (car (send *robot* :links)) :copy-worldcoords))
      (setq l-pos (send waist-coords :inverse-transform-vector l-pos-wrt)) ;; transformed BODY frame
      (setq r-pos (send waist-coords :inverse-transform-vector r-pos-wrt)) ;; transformed BODY frame

      (setq point-l (instance  geometry_msgs::Point32 :init)) ;; left
      (send point-l :x (elt l-pos 0))
      (send point-l :y (elt l-pos 1))
      (send point-l :z (elt l-pos 2))

      (setq point-r (instance  geometry_msgs::Point32 :init)) ;; right
      (send point-r :x (elt r-pos 0))
      (send point-r :y (elt r-pos 1))
      (send point-r :z (elt r-pos 2))
      
      (format t "point-l=~A~%" point-l)
      ;; (setq points-l-list (append (list point-l) points-l-list))
      ;; (setq points-r-list (append (list point-r) points-r-list))
      (setq points-l-list (list-insert point-l 100 points-l-list))
      (setq points-r-list (list-insert point-r 100 points-r-list))
      
      )
    (send req :l_points points-l-list)
    (send req :r_points points-r-list)
    (send frame_id :data "BODY")
    (send req :frame_id frame_id)
    (format t "l-list = ~A~%" points-l-list)
    (format t "candidates-coords-list = ~A~%" candidates-coords-list)    
    (setq res (ros::service-call "display_grasp_candidates" req t))
    )
  )

(defun cb (req)
  (format t "in cb euslisp~%")
  (if (send req :flag) ;; True: return cnadidates
      (send-grasp-candidates :hands-interval (send req :data))

    (progn ;; False solve ik
      (format t "req = ~A~%" (elt (send req :data) 0))
      ;; (setq coords-list (list (elt candidates-coords-list (* 2 (elt (send req :data) 0))) (elt candidates-coords-list (+ (* 2 (elt (send req :data) 0)) 1))))
      (setq coords-list (list (elt (elt candidates-coords-list (elt (send req :data) 0)) 0) (elt (elt candidates-coords-list (elt (send req :data) 0)) 1)))
      (format t "coords-list = ~A~% " coords-list)
      (send *robot* :angle-vector (send *ri* :state :angle-vector))
      (reset-coords)
      ;; (ik-opt (elt coords-list 0) (elt coords-list 1) :only-kinematics nil  :optimize-torque? nil :fix-contact? 2 :avoid-collision? t :apply-external-force? nil :wait? t :rleg-pos-shift #f(0 0 0) :lleg-pos-shift #f(0 0 0)   :set-unique-kin-scale? t :obst? nil)
      ;; (send *robot-env* :dissoc-root-virtual)
      (ik (elt coords-list 0) (elt coords-list 1))

      (send-pose-and-move-on-foot :time 8000 :send? t :fix-contact? 2   :look-at-object? nil)
      )
    )
  (format t "receive trigger_with_data")
  )

(defun ik(larm rarm)
  (send *robot* :fullbody-inverse-kinematics
        (list (send *robot* :rleg :end-coords :copy-worldcoords)
              (send *robot* :lleg :end-coords :copy-worldcoords)
              larm
              rarm
              )
        :move-target
        (list (send *robot* :rleg :end-coords)
              (send *robot* :lleg :end-coords)
              (send *robot* :larm :end-coords)
              (send *robot* :rarm :end-coords))
        :link-list
        (list (send *robot* :link-list (send *robot* :rleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :lleg :end-coords :parent))
              (send *robot* :link-list (send *robot* :larm :end-coords :parent))
              (send *robot* :link-list (send *robot* :rarm :end-coords :parent)))
        :translation-axis (list t t t t)
        :rotation-axis (list t t :z :z)
        :target-centroid-pos (midpoint 0.5
                                       (send *robot* :rleg :end-coords :worldpos)
                                       (send *robot* :lleg :end-coords :worldpos))
        :cog-translation-axis :z)
  ;; (send *obj* :locate (send *robot* :centroid) :world)
  (send *irtviewer* :draw-objects))

(ros::advertise-service "joy_trigger" rm_debris_vision::TriggerWithData #'cb)
(do-until-key
  (ros::spin-once)
 )
