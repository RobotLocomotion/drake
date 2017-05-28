Copyright 2016-2017 Toyota Research Institute.  All rights reserved.  


# Drake Docker containers
There are two Docker containers provided that can build Drake in isolated
Ubuntu 16.04 environments.
# Building
$ cd <drake-root-dir>  

If you have the Nvidia drivers installed:  
$ docker build -t drake -f tools/docker/Dockerfile.nvidia .  
  
If you are using open source graphics drivers (Nouveau, Intel, ...):  
$ docker build -t drake -f tools/docker/Dockerfile.opensource .    
  
# Running
## Passive Acrobot Simulation
### Nvidia drivers:  (requires nvidia-docker plugin)  
$ xhost +local:root; nvidia-docker run -ti --rm -e DISPLAY \  
-e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \  
--privileged drake; xhost -local:root  
  
### Open source drivers:  
$ xhost +local:root; docker run -ti --rm -e DISPLAY \  
-e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \  
--privileged drake; xhost -local:root  
  
Note: The --privileged argument is only necessary under security enhanced
linux.

## Enter An Interactive Shell
An arbitrary command can be passed to the docker image by appending the
argument to the above commands. Type bash at the end to enter a bash shell in
the docker image.  
$ xhost +local:root; docker run -ti --rm -e DISPLAY \  
-e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix \  
--privileged drake bash; xhost -local:root  

