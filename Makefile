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
	@[ -d data/rospersistent-delta ] || mkdir -p data/rospersistent-delta
	@[ -d data/rospersistent-epsilon ] || mkdir -p data/rospersistent-epsilon
	@[ -d data/rospersistent-zeta ] || mkdir -p data/rospersistent-zeta
	@[ -d data/rospersistent-eta ] || mkdir -p data/rospersistent-eta
	@[ -d data/rospersistent-theta ] || mkdir -p data/rospersistent-theta
	@[ -d data/rospersistent-iota ] || mkdir -p data/rospersistent-iota
	@[ -d data/rospersistent-kappa ] || mkdir -p data/rospersistent-kappa

	# Add correct local directories path to compose file
	@sed "s|ROSPERSISTENT-PATH|$$(pwd)|" docker-compose-server.yaml > .docker-compose.yaml
	# Starting containers... 
	docker-compose -f .docker-compose.yaml up -d

.PHONEY: restart-server
restart-server:
	# Create local directories for storing data
	@[ -d data/rospersistent-alpha ] || mkdir -p data/rospersistent-alpha
	@[ -d data/rospersistent-beta ] || mkdir -p data/rospersistent-beta
	@[ -d data/rospersistent-gamma ] || mkdir -p data/rospersistent-gamma
	@[ -d data/rospersistent-delta ] || mkdir -p data/rospersistent-delta
	@[ -d data/rospersistent-epsilon ] || mkdir -p data/rospersistent-epsilon
	@[ -d data/rospersistent-zeta ] || mkdir -p data/rospersistent-zeta
	@[ -d data/rospersistent-eta ] || mkdir -p data/rospersistent-eta
	@[ -d data/rospersistent-theta ] || mkdir -p data/rospersistent-theta
	@[ -d data/rospersistent-iota ] || mkdir -p data/rospersistent-iota
	@[ -d data/rospersistent-kappa ] || mkdir -p data/rospersistent-kappa

	# Add correct local directories path to compose file
	@sed "s|ROSPERSISTENT-PATH|$$(pwd)|" docker-compose-server.yaml > .docker-compose.yaml
	# Restarting containers... 
	docker-compose -f .docker-compose.yaml up -d $(containers)

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
