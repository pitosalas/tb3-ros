.PHONEY: start
start:
	@[ -d rospersistent ] || mkdir rospersistent
	@sed "s|ROSPERSISTENT-PATH|$$(pwd)\/rospersistent|" docker-compose.yaml > .docker-compose.yaml 
	docker-compose -f .docker-compose.yaml up -d

.PHONEY: sol
sol:
	@[ -d rospersistent ] || mkdir rospersistent
	@sed "s|ROSPERSISTENT-PATH|$$(pwd)\/rospersistent|" docker-compose-sol.yaml > .docker-compose.yaml 
	docker-compose -f .docker-compose.yaml up -d

.PHONEY: start-server
start-server:
	# Create local directories for storing data
	@[ -d data/rospersistent-alpha ] || mkdir -p data/rospersistent-alpha
	@[ -d data/rospersistent-beta ] || mkdir -p data/rospersistent-beta
	@[ -d data/rospersistent-gamma ] || mkdir -p data/rospersistent-gamma
	# Add correct local directories path to compose file
	@sed "s|ROSPERSISTENT-PATH|$$(pwd)|" docker-compose-sol.yaml > .docker-compose.yaml 
	docker-compose -f .docker-compose.yaml up -d

.PHONEY: stop
stop:
	docker-compose -f .docker-compose.yaml down

.PHONEY: build
build:
	docker build -t cosi119/tb3-ros:latest --no-cache .

.PHONEY: fix
fix:
	docker network prune -f
	docker container prune -f
