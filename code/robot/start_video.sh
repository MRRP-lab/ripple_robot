#!/bin/bash
# Start SRT video stream from USB webcam
# Hardware-encoded H.264 @ 720p30fps for low latency

# Video device (change if your webcam is on a different device)
VIDEO_DEVICE="/dev/video0"

# Stream settings
RESOLUTION="480x272"
FRAMERATE="30"
BITRATE="2M"
SRT_PORT="9000"

# Check if device exists
if [ ! -e "$VIDEO_DEVICE" ]; then
    echo "Error: Video device $VIDEO_DEVICE not found"
    echo "Available devices:"
    ls -l /dev/video*
    exit 1
fi

echo "Starting SRT video stream..."
echo "Device: $VIDEO_DEVICE"
echo "Resolution: $RESOLUTION @ ${FRAMERATE}fps"
echo "Listening on: srt://0.0.0.0:$SRT_PORT"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Start SRT listener with H.264 encoding
ffmpeg -f v4l2 \
  -input_format mjpeg \
  -video_size $RESOLUTION \
  -framerate $FRAMERATE \
  -i $VIDEO_DEVICE \
  -c:v libx264 \
  -preset ultrafast \
  -tune zerolatency \
  -b:v $BITRATE \
  -maxrate $BITRATE \
  -bufsize 1M \
  -g 30 \
  -keyint_min 30 \
  -pix_fmt yuv420p \
  -f mpegts \
  "srt://0.0.0.0:$SRT_PORT?mode=listener"
