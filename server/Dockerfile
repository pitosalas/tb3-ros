FROM golang:1.14.3-alpine AS build

ARG APP
ARG COMMIT_SHA

WORKDIR /src

COPY . .
RUN go mod download

RUN go get -u github.com/rakyll/statik
RUN go generate ./...

RUN mkdir darwin linux

RUN GOOS=darwin GOARCH=amd64 \
    go build -o darwin/${APP} \
        -ldflags="-X 'main.BuildVersion=${COMMIT_SHA}'" \
        ./cmd/${APP}/*.go

RUN GOOS=linux GOARCH=amd64 \
    go build -o linux/${APP} \
        -ldflags="-X 'main.BuildVersion=${COMMIT_SHA}'" \
        ./cmd/${APP}/*.go

FROM scratch AS export
COPY --from=build /src/darwin /darwin
COPY --from=build /src/linux /linux
