# Drake Docker containers
There are two Docker containers provided that can build Drake in isolated
Ubuntu 16.04 environments.
# Building
$ cd <drake-root-dir>  

If you have the Nvidia drivers installed:  
$ docker build -t drake -f tools/docker/Dockerfile.nvidia .  
  
If you are using open source graphics drivers:  
$ docker build -t drake -f tools/docker/Dockerfile.opensource .    
  
# Running
## Passive Acrobot Simulation
Nvidia drivers:  
$ xhost +local:root; nvidia-docker run -ti --rm --env=DISPLAY
--env='QT_X11_NO_MITSHM=1' --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw'
--privileged drake
  
Open source drivers:  
$ xhost +local:root; docker run -ti --rm --env=DISPLAY
--env='QT_X11_NO_MITSHM=1' --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw'
--privileged drake

##Enter An Interactive Shell
An arbitrary command can be passed to the docker image by appending an argument
to the above commands. Type bash at the end to enter a bash shell in the docker
image.
