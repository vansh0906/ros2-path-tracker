# ROS 2 Humble image that builds the package, runs the unit tests, then runs the simulation.
#   docker build -t ros2-path-tracker .
#   docker run --rm -v "$PWD/results:/results" ros2-path-tracker controller:=lqr csv_path:=/results/lqr.csv
FROM ros:humble
SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY path_tracker ./src/path_tracker
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/*
# build, then fail the image build if any unit test fails
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    colcon test --packages-select path_tracker --event-handlers console_direct+ && \
    colcon test-result --verbose
COPY scripts/run.sh /run.sh
RUN chmod +x /run.sh
ENTRYPOINT ["/run.sh"]
