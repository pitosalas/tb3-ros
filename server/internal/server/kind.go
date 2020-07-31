package server

import (
	"net"
)

type Services struct {
	DesktopList []*Service
	Relay       *Service
	Network     *Network
}

type Service struct {
	ServiceName   string
	ContainerName string
	HostName      string
	Port          map[string]int
	Secret        map[string]string
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
