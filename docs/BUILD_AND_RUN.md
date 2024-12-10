# Build and run
## Local build
Prequistances:
```bash
sudo apt-get install libyaml-cpp-dev
```
Build:
```bash
mkdir -p ~/wisevision_action_executor_ws/src && cd ~/wisevision_action_executor_ws/src
git clone git@github.com:wise-vision/wisevision_action_executor.git
cd wisevision_action_executor
vcs import --recursive < wisevision_action_executor.repos
cd ../..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to wisevision_action_executor
```
Run:
```bash
source install/setup.bash
ros2 run wisevision_action_executor automatic_action_service
```

## Docker build and run
Prequistances: Clone repository:
```bash
mkdir -p ~/wisevision_action_executor_ws/src && cd ~/wisevision_action_executor_ws/src
git clone git@github.com:wise-vision/wisevision_action_executor.git
```
In folder `wisevision_action_executor` run:
```bash
docker compose up 
```