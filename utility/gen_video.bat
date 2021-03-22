@ECHO OFF

::force overwrite
ffmpeg.exe -y ^
-r 60 ^
-i ../Screenshots/%%3d.jpg ^
-c:v libx264 -qp 1 -profile:v high444 -preset fast -pix_fmt yuv420p ^
result.mp4
ECHO DONE!