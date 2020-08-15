# This will create a dockerfile and then copy the source files from the context
# directory.
# Requires that base.dockerfile is also built (use make base)
FROM pirhanabot:base-ros


## Copy From Host For Worksapce Setup #########################################
ENV HOME=/root
RUN mkdir ${HOME}/catkin_ws
RUN mkdir ${HOME}/catkin_ws/src
# Copy the packages
COPY board_interface/ ${HOME}/catkin_ws/src/board_interface/
COPY watchdog/ ${HOME}/catkin_ws/src/watchdog/

# Copy a catkin helper because catkin is bad
COPY catkin/Makefile ${HOME}/catkin_ws/

## Copy Scripts ###############################################################
COPY Docker/scripts-local/* /scripts/

## Startup ####################################################################
# At run, this will execute any command set up previously by echoing into it.
ENTRYPOINT ["/bin/bash"]
CMD ["/scripts/startup.sh"]
