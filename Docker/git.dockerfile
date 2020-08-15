# This will create a dockerfile and then clone the repository put in the 
# environment variables.
# Requires that base.dockerfile is also built (use make base)
FROM pirhanabot:base-ros

## Git Clone For Worksapce Setup ##############################################
ENV WORKSPACE_GIT="https://github.com/CuUwrRobotics/PiranhaBot.git"
ENV GIT_BRANCH="master"
ENV WORKSPACE_GIT_NAME="PiranhaBot"

## Copy Scripts ###############################################################
COPY Docker/scripts-git/* /scripts/

## Startup ####################################################################
# At run, this will execute any command set up previously by echoing into it.
ENTRYPOINT ["/bin/bash"]
CMD ["/scripts/startup.sh"]
