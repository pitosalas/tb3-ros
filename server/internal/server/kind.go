package server

import (
	"net"
)

// Services represents a tb3-ros server.
type Services struct {
	DesktopList []*Service
	Proxy       *Service
	Relay       *Service
	Network     *Network
	Config      *ConfigFile
}

// Service represents a service in the compose file.
type Service struct {
	Name          string
	Owner         string
	ServiceName   string
	ContainerName string
	HostName      string
	Port          map[string]int
	Secrets       map[string]string
	Files         map[string]string
	Hosts         map[string]string
	Network       *Network
	IPv4Address   net.IP
	Volume        *Volume
}

// Network represents the network info of a Service.
type Network struct {
	Name    string
	Subnet  *net.IPNet
	Gateway net.IP
}

// Volume represents the volume info of a Service.
type Volume struct {
	Name string
	Path string
}
