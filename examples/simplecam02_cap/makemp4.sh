#!/bin/sh
ffmpeg -pattern_type glob -r 5 -i "*.png" -s 640x480 -vcodec libx264 out.mp4
