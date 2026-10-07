# fr3_datahive: collect FR3 episodes for DataHive (ROS 1 Noetic container).
#
#   make build                  build the image
#   make run | shell | stop     start the container / attach a shell / remove it
#   make init                   one-time: datahive init (lab id + Hugging Face token)
#   make profile                install the FR3 robot profile into samples/ (never overwrites)
#   make interface              web GUI on http://localhost:8000 (Runner + Annotate)
#   make collect                EVERYTHING in one command: robot + cameras + controller + joystick
#                               + GUI + recorder. MODE=manual|auto, ARGS="robot_ip:=... front_device:=..."
#   make bringup                franka_control + gripper + cameras, ARGS="robot_ip:=172.16.0.2"
#   make teleop                 impedance controller + joystick (ARGS="mode:=none" for controller only)
#   make engage | disengage     enable / disable teleop motion
#   make home                   move the arm to the home pose
#   make cameras-list           show the /dev/v4l paths to use in ARGS="front_device:=..."
#   make manual                 record episodes to files, then upload them in the Runner (Manual)
#   make auto                   record the trials the Runner sends ("Send to robot")
#   make check                  datahive check on every episode; SAMPLES=/data/recordings for manual ones
#   make exec CMD="rostopic list"

IMAGE     ?= fr3-datahive:noetic
CONTAINER ?= fr3-datahive
PORT      ?= 8000
DATAHIVE_SPEC ?= datahive-tools[video]>=0.1.7
HERE      := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

HOST_DIRS := $(HERE)/samples $(HERE)/recordings $(HERE)/datahive-config

MOUNTS := -v $(HERE)/samples:/data/samples \
          -v $(HERE)/recordings:/data/recordings \
          -v $(HERE)/datahive-config:/root/.datahive \
          -v $(HERE)/ros/fr3_datahive:/root/catkin_ws/src/fr3_datahive \
          -v $(HERE)/ros/fr3_teleop:/root/catkin_ws/src/fr3_teleop \
          -v $(HERE)/ros/fr3_control:/root/catkin_ws/src/fr3_control

RUN_FLAGS := --network host \
             --cap-add=SYS_NICE --ulimit rtprio=99 \
             --device-cgroup-rule='c 81:* rmw' \
             --device-cgroup-rule='c 189:* rmw' \
             --device-cgroup-rule='c 13:* rmw' \
             -v /dev:/dev -v /run/udev:/run/udev:ro

ENTRY := /usr/local/bin/entrypoint.sh
TTY   := $(shell [ -t 0 ] && echo -it || echo -i)
EXEC   = docker exec $(TTY) $(CONTAINER) $(ENTRY)
esc    = $(subst ','"'"',$(1))

.PHONY: all build .image-present rebuild run shell stop clean status init profile interface collect bringup teleop engage disengage home cameras-list manual auto check exec

all: run shell

build:
	docker build --build-arg DATAHIVE_SPEC='$(DATAHIVE_SPEC)' -t $(IMAGE) $(HERE)

.image-present:
	@docker image inspect $(IMAGE) >/dev/null 2>&1 || $(MAKE) build

rebuild: stop
	docker build --pull --build-arg DATAHIVE_SPEC='$(DATAHIVE_SPEC)' -t $(IMAGE) $(HERE)
	@$(MAKE) run

run: .image-present
	@mkdir -p $(HOST_DIRS)
	@if [ -z "$$(docker ps -q -f name='^$(CONTAINER)$$')" ]; then \
		docker rm -f $(CONTAINER) >/dev/null 2>&1 || true; \
		echo ">> starting $(CONTAINER)"; \
		docker run -d --name $(CONTAINER) $(RUN_FLAGS) $(MOUNTS) $(IMAGE) sleep infinity >/dev/null; \
	else \
		echo ">> $(CONTAINER) already running"; \
	fi

shell: run
	@$(EXEC) bash

stop:
	@docker rm -f $(CONTAINER) >/dev/null 2>&1 || true

clean: stop
	@docker rmi $(IMAGE) >/dev/null 2>&1 || true

status:
	@docker ps -a --filter name='^$(CONTAINER)$$' --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'

init: run
	@$(EXEC) datahive init

profile: run
	@$(EXEC) bash -lc 'test -f /data/samples/robot_profile.yaml \
	  && echo "samples/robot_profile.yaml already exists; edit it, it is not overwritten" \
	  || { cp "$$(rospack find fr3_datahive)/config/robot_profile.yaml" /data/samples/robot_profile.yaml \
	       && echo "wrote samples/robot_profile.yaml -- review the lines marked EDIT"; }'

interface: run
	@echo ">> open http://localhost:$(PORT)"
	@$(EXEC) datahive interface --port $(PORT) --samples /data/samples

MODE ?= manual

collect: run
	@$(EXEC) bash -lc 'test -f /data/samples/robot_profile.yaml || { cp "$$(rospack find fr3_datahive)/config/robot_profile.yaml" /data/samples/robot_profile.yaml; echo ">> wrote samples/robot_profile.yaml from the FR3 template -- check the lines marked EDIT"; }; \
	  datahive interface --port $(PORT) --samples /data/samples > /tmp/datahive-interface.log 2>&1 & IPID=$$!; trap "kill $$IPID 2>/dev/null" EXIT; \
	  echo ">> GUI: http://localhost:$(PORT)   mode: $(MODE)   engage teleop with the pad Start button"; \
	  roslaunch fr3_datahive collect.launch mode:=$(MODE) $(call esc,$(ARGS))'

bringup: run
	@$(EXEC) bash -lc 'roslaunch fr3_datahive bringup.launch $(call esc,$(ARGS))'

teleop: run
	@echo ">> motion stays gated until you run: make engage"
	@$(EXEC) bash -lc 'roslaunch fr3_teleop teleop.launch $(call esc,$(ARGS))'

engage: run
	@$(EXEC) bash -lc 'rosservice call /teleop/engage'

disengage: run
	@$(EXEC) bash -lc 'rosservice call /teleop/disengage'

home: run
	@$(EXEC) bash -lc 'roslaunch fr3_datahive home.launch $(call esc,$(ARGS))'

cameras-list: run
	@$(EXEC) bash -lc 'v4l2-ctl --list-devices || true; echo; ls -l /dev/v4l/by-id/ /dev/v4l/by-path/ 2>/dev/null || echo "no /dev/v4l entries -- are the cameras plugged in?"'

manual: run
	@$(EXEC) bash -lc 'roslaunch fr3_datahive record.launch mode:=manual $(call esc,$(ARGS))'

auto: run
	@$(EXEC) bash -lc 'roslaunch fr3_datahive record.launch mode:=auto $(call esc,$(ARGS))'

check: run
	@$(EXEC) datahive check --samples $(or $(SAMPLES),/data/samples)

exec: run
	@$(EXEC) bash -lc '$(call esc,$(CMD))'
