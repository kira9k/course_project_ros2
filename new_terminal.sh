#!/bin/sh

CONTAINER_NAME="ros2_course_project"
CONTAINER_ID=$(docker ps --filter "ancestor=ros2_course_project" --format "{{.ID}}")
COMMANDS=". install/setup.bash"  
LOG_FILE="youbot_console.log" 


docker exec -it "$CONTAINER_ID" /bin/bash -c "$COMMANDS && exec /bin/bash"
