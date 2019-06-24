export DISPLAY=:0
(vlc ${ROS_WORKSPACE}/src/pioneer_shr/resource/warning-video.mkv; killall -9 vlc; eog --fullscreen ${ROS_WORKSPACE}/src/pioneer_shr/resource/big_white_face.jpg) &

