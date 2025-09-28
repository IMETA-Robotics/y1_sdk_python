#!/bin/bash

DEV_CONTAINER="y1_sdk_ubuntu22_04"

function remove_container_if_exists() {
	local container="$1"
	if docker ps -a --format '{{.Names}}' | grep -q "${container}"; then
		echo "Removing existing container: ${container}"
		docker stop "${container}" >/dev/null
		docker rm -v -f "${container}" 2>/dev/null
	fi
}

if ! [[ -x "$(command -v docker)" ]]; then
	echo "docker not exist, please install docker first."
	exit 1
fi

docker build -t y1_sdk_ubuntu22_04 "$(pwd)/docker"

echo "Remove existing y1_sdk_ubuntu22_04 container ..."
remove_container_if_exists ${DEV_CONTAINER}

docker run -itd \
		--name y1_sdk_ubuntu22_04 \
		--user=$(id -u $USER):$(id -g $USER) \
		--network=host \
		--gpus=all \
		--privileged \
		--workdir "/home/$USER/y1_sdk_ubuntu22_04" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="DISPLAY" \
		--volume=$(pwd):"/home/$USER/y1_sdk_ubuntu22_04" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="/etc/group:/etc/group:ro" \
		--volume="/etc/passwd:/etc/passwd:ro" \
		--volume="/etc/shadow:/etc/shadow:ro" \
		--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="/dev:/dev" \
		y1_sdk_ubuntu22_04:latest

echo "Congratulations! You have successfully finished setting up ros2 humble Dev Environment."
echo "To login into the newly created y1_sdk_ubuntu22_04 container, please run the following command:"
echo "bash docker/dev_into.sh"
echo "Enjoy!"