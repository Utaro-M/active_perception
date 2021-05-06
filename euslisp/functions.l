;; 重心位置の比較によりモデルとセンサ値の一致を確認する
;; 重心位置のズレがthre以下ならt 以上ならnilをかえす

(require "../../rm_debris/rm_board/euslisp/functions.l")

(defun check-fitting
    (plane
     &key
     (thre 100)
     (thre-angle 10)
     (larm-force (low-pass-online "force" "off" :larm :times 10 :sleep 100 :flag nil))
     (rarm-force (low-pass-online "force" "off" :rarm :times 10 :sleep 100 :flag nil))
     (larm-moment (low-pass-online "moment" "off" :larm :times 10 :sleep 100 :flag nil))
     (rarm-moment (low-pass-online "moment" "off" :rarm :times 10 :sleep 100 :flag nil))     )
  
  (let*
      ((com-from-model (elt plane 1))
       (larm-coords (send *robot* :larm :end-coords :copy-worldcoords)) (l-pos (send larm-coords :worldpos))
       (rarm-coords (send *robot* :rarm :end-coords :copy-worldcoords)) (r-pos (send rarm-coords :worldpos))       
       (lf-z (elt larm-force 2))(rf-z (elt rarm-force 2))
       (lm-z (elt larm-moment 2))(rm-z (elt rarm-moment 2))
       (com-y-from-value #f(0 0 0)) (com-y-from-model #f(0 0 0))
       (line (make-line l-pos r-pos))
       (angle 0)
       )

    ;; get com from force value
    (setq com-y-from-value (scale (/ 1.0 (+ lf-z rf-z)) (v+ (scale lf-z l-pos) (scale rf-z r-pos))))
    (format t "com-y-from-value = ~A~%" com-y-from-value)

    ;; get com from model
    (format t "com-from-model = ~A~%" com-from-model)
    (setq com-y-from-model (send line :point (send line :foot com-from-model)))
    (format t "com-y-from-model = ~A~%" com-y-from-model)

    ;;angle between line from hands-pos and normale-vec from model
    (setq angle (rad2deg (v.  (send line :direction) normal-vec)))
    (format t "cos = ~A~%" angle)

    ;; check
    (if (and (< (norm (v- com-y-from-model com-y-from-value)) thre) (< angle thre-angle))
        (return-from check-fitting t)
      (return-from check-fitting nil))
    )
  )
