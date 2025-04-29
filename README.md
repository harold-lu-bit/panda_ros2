# ROS 2 integration for Franka Emika research robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs

## Install with Docker

```shell
# launch docker
cd franka_ros2/docker
docker compose up -d
# enter container
docker exec -it franka_panda_ros2 /bin/bash
# build inside docker
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```