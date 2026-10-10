ffmpeg -f v4l2 -framerate 30 -video_size 640x480 \
-i /dev/video0 \
-vcodec libx264 -preset ultrafast -tune zerolatency \
-f rtsp rtsp://localhost:8554/mystream
