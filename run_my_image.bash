XAUTH_LIST=$(xauth list)

docker run -ti \
       --net=host \
       -e DISPLAY \
       -v /tmp/.X11-unix \
       --runtime=nvidia \
       morse_simulation_morse bash -c "xauth add $XAUTH_LIST && bash"
