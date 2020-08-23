GO111MODULES=on
APP?=serverctl

.PHONY: build
## build: build the application
build:
	@echo "Building..."
	@goreleaser build --snapshot --rm-dist

.PHONY: run
## run: runs go run main.go
run:
	go run -race cmd/${APP}/*.go

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

.PHONY: help
## help: Prints this help message
help:
	@echo "Usage: \n"
	@sed -n 's/^##//p' ${MAKEFILE_LIST} | column -t -s ':' |  sed -e 's/^/ /'
