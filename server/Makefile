GO111MODULES=on
APP?=serverctl
COMMIT_SHA=$(shell git rev-parse --short HEAD)

.PHONY: build
## build: build the application
build: clean
	@echo "Building..."
	@go generate ./...
	@go build -ldflags="-X 'main.BuildVersion=${COMMIT_SHA}'" \
		-o bin/${APP} \
		cmd/${APP}/*.go

.PHONY: run
## run: runs go run main.go
run:
	go run -race cmd/${APP}/*.go

.PHONY: clean
## clean: cleans the binary
clean:
	@echo "Cleaning"
	@go clean

.PHONY: test
## test: runs go test with default values
test:
	go test -v -count=1 -race ./...

check: staticcheck

staticcheck:
	go run honnef.co/go/tools/cmd/staticcheck -- $$(go list ./... | grep -v tempfork)

.PHONY: setup
## setup: setup go modules
setup:
	@go mod init \
		&& go mod tidy \
		&& go mod vendor

.PHONY: docker-build
## docker-build: builds the stringifier docker image to registry
docker-build: build
	DOCKER_BUILDKIT=1 docker build \
		--build-arg APP=${APP} \
		--build-arg COMMIT_SHA=${COMMIT_SHA} \
		-o bin .

.PHONY: help
## help: Prints this help message
help:
	@echo "Usage: \n"
	@sed -n 's/^##//p' ${MAKEFILE_LIST} | column -t -s ':' |  sed -e 's/^/ /'
