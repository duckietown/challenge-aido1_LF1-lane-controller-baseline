# Definition of Submission container

# We start from the AIDO1_LF1 ROS template
FROM duckietown/challenge-aido1_lf1-template-ros:v3

RUN ["cross-build-start"]

# let's copy all our solution files to our workspace
# if you have more file use the COPY command to move them to the workspace
COPY solution.py ./

# For ROS Agent - Additional Files
COPY rosagent.py ./
COPY lf_slim.launch ./
COPY catkin_ws ./catkin_ws/

COPY controller.py /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/
RUN chmod 777 /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/controller.py

# Copy new tuned controller files
# RUN rm /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/lane_controller_node.py
COPY lane_controller_node.py /home/software/catkin_ws/src/10-lane-control/lane_control/scripts
# COPY lane_controller_node.launch /home/software/catkin_ws/src/10-lane-control/lane_control/launch

RUN chmod 777 /home/software/catkin_ws/src/10-lane-control/lane_control/scripts/lane_controller_node.py
# RUN chmod 777 /home/software/catkin_ws/src/10-lane-control/lane_control/launch/lane_controller_node.launch

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && cd /home/software/catkin_ws && catkin_make install --pkg lane_control"

RUN pip install ruamel.yaml

# Probably not necessary
# RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j -C /home/software/catkin_ws"

# Uncomment these to build your own **SELF-CONTAINED** catkin_ws - warning: not the fastest!
# RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make -j -C catkin_ws/"
# And adding it to the path
# RUN echo "source $(pwd)/catkin_ws/devel/setup.bash" >> ~/.bashrc

# DO NOT MODIFY: your submission won't run if you do
ENV DUCKIETOWN_SERVER=evaluator

RUN ["cross-build-end"]
ENTRYPOINT ["qemu3-arm-static"]

# let's see what you've got there...
CMD ["/bin/bash", "-ci", "./solution.py"]
