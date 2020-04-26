.PHONEY: start
start:
	@[ -d rospersistent ] || mkdir rospersistent
	@erb docker-compose.yaml > .docker-compose.yaml
	docker-compose -f .docker-compose.yaml up -d

.PHONEY: stop
stop:
	docker-compose -f .docker-compose.yaml down

.PHONEY: build
build:
	docker build -t tb3-ros:latest --no-cache .

.PHONEY: fix
fix:
	docker network prune -f
	docker container prune -f