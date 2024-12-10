FROM wisevision/ros_with_wisevision_msgs:humble

WORKDIR /root/wisevision_action_executor_ws

COPY  . /root/wisevision_action_executor_ws/src/wisevision_action_executor

RUN apt-get update && \
    apt-get install -y libyaml-cpp-dev && \
    sudo rosdep fix-permissions && \
    rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble




SHELL ["/bin/bash", "-c"]

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
source /root/wisevision_msgs_ws/install/setup.bash && \
colcon build --symlink-install"

ENTRYPOINT ["/bin/bash", "-c", "source install/setup.bash && ros2 run wisevision_action_executor automatic_action_service"]
RUN echo 'source install/setup.bash && ros2 run wisevision_action_executor automatic_action_service' >> ~/.bashrc