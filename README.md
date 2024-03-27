# Dockerfile for ROS Rolling and Gazebo Garden

- 필요한 Dockerfile은 Dockerhub에서 버전 설명되있는거 클릭하면 깃허브에서 Dokerfile볼수 있음
- 필요한 Dockerfile을 만들어서 빌드하고 실행하면 됨

  ```bash
  docker build -f rolling-harmonic.dockerfile -t ros-gazebo:latest .
  ```

- 맥

```bash
xhost +${HOSTNAME}
docker run -it --privileged --env="DISPLAY=host.docker.internal:0" -v /dev:/dev --group-add dialout -v /tmp/.X11-unix:/tmp/.X11-unix:ro ros-gazebo
```

- 리눅스

```bash
docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY ros-gazebo:latest
```