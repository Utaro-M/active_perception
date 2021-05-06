#!/usr/bin/env roseus

(ros::load-ros-manifest "rm_debris_vision")
(require "../../../rm_debris/rm_board/euslisp/functions.l")
(require "../../../rm_debris/rm_board/euslisp/client/make_plane_for_opt.l ")
(ros::roseus "display_grasp_candidates_server")
(ros::wait-for-service "display_grasp_candidates")

(defun send-grasp-candidates(&key (hands-interval (list  300 500 700)))
  (setq req (instance rm_debris_vision::GraspCandidatesRequest :init))
  (let*
      ((lis (list))
       (points-l-list (list))
       (points-r-list (list))
       (coords-tmp )
       (l-pos #f(0 0 0))
       (r-pos #f(0 0 0)))
    
    (dotimes (i (length hands-interval))
      (setq coords-tmp (auto-set-reaching-targets-each-hand :pos-shift (elt hands-interval i)))
      ;; (format t "coords-tmp = ~A~%" coords-tmp)
      (setq l-pos (send (send (elt coords-tmp 0) :copy-worldcoords) :worldpos))
      (setq r-pos (send (send (elt coords-tmp 1) :copy-worldcoords) :worldpos))
      (setq l-pos #f( 400 400 800))
      ;; (setq r-pos #f( 1000 1000 300))      
      (setq point-l (instance  geometry_msgs::Point32 :init)) ;; left
      (send point-l :x (elt l-pos 0))
      (send point-l :y (elt l-pos 1))
      (send point-l :z (elt l-pos 2))

      (setq point-r (instance  geometry_msgs::Point32 :init)) ;; right
      (send point-r :x (elt r-pos 0))
      (send point-r :y (elt r-pos 1))
      (send point-r :z (elt r-pos 2))
      (format t "point-l=~A~%" point-l)
      (setq points-l-list (append (list point-l) points-l-list))
      (setq points-r-list (append (list point-r) points-r-list))
      
      )
    (send req :l_points points-l-list)
    (send req :r_points points-r-list)
    (format t "l-list = ~A~%" points-l-list)
    (setq res (ros::service-call "display_grasp_candidates" req t))
    
    )
  )

