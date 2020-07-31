package server

import (
	"net"
)

type Services struct {
	DesktopList []*Service
	Proxy       *Service
	Relay       *Service
	Network     *Network
	Config      *ConfigFile
}

type Service struct {
	Name          string
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

type Network struct {
	Name    string
	Subnet  *net.IPNet
	Gateway net.IP
}

type Volume struct {
	Name string
	Path string
}
