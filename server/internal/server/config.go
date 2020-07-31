package server

import (
	"fmt"
	"io/ioutil"
	"net"
	"path/filepath"

	"github.com/bxcodec/faker/v3"
	"gopkg.in/yaml.v2"
)

type ConfigFile struct {
	ServerName string `yaml:"server_name"`
	Desktop    struct {
		Names    []string `yaml:",flow"`
		Security struct {
			RandomPassword bool   `yaml:"random_password"`
			Password       string `yaml:",omitempty"`
		}
		Data struct {
			Directory string `yaml:"directory"`
		}
	}
	Relay struct {
		AuthKey  string `yaml:"authkey"`
		DockerIP string `yaml:"docker_ip"`
	}
	Network struct {
		Subnet  string `yaml:"subnet"`
		Gateway string `yaml:"gateway"`
	}
}

func ParseConfig(path string) (*Services, error) {
	var configFile ConfigFile

	f, err := ioutil.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("unable to read config file %v", err)
	}

	err = yaml.Unmarshal(f, &configFile)
	if err != nil {
		return nil, fmt.Errorf("failed to parse config file %v", err)
	}

	return configFile.Services()
}

func (c *ConfigFile) Services() (*Services, error) {
	network, err := c.network()
	if err != nil {
		return nil, fmt.Errorf("failed to parse network %v", err)
	}

	relay, err := c.relay(network)
	if err != nil {
		return nil, fmt.Errorf("failed to parse relay %v", err)
	}

	desktopList, err := c.desktopList(network)
	if err != nil {
		return nil, fmt.Errorf("failed to parse desktop %v", err)
	}

	return &Services{
		DesktopList: desktopList,
		Relay:       relay,
		Network:     network,
	}, nil
}

func (c *ConfigFile) desktopList(network *Network) ([]*Service, error) {
	desktopList := make([]*Service, 0, len(c.Desktop.Names))

	id := 1
	for _, n := range c.Desktop.Names {
		name := fmt.Sprintf("%s-%s", c.ServerName, n)

		var password string
		if c.Desktop.Security.RandomPassword {
			password = faker.Password()
		} else {
			password = c.Desktop.Security.Password
		}

		vName := fmt.Sprintf("rospersistent-%s", name)
		vPath, err := volumePath(c.Desktop.Data.Directory, vName)
		if err != nil {
			return nil, fmt.Errorf("failed to create desktop list %v", err)
		}

		s := &Service{
			ServiceName:   name,
			ContainerName: name,
			HostName:      name,
			Port: map[string]int{
				"ssh":    2220 + id,
				"vscode": 8080 + id,
				"vnc":    6080 + id,
			},
			Secret: map[string]string{
				"GW_IP":    network.Gateway.String(),
				"RELAY_IP": net.ParseIP(c.Relay.DockerIP).To4().String(),
				"PASSWORD": password,
			},
			Network: network,
			Volume: &Volume{
				Name: vName,
				Path: vPath,
			},
		}

		desktopList = append(desktopList, s)
	}

	return desktopList, nil
}

func (c *ConfigFile) relay(network *Network) (*Service, error) {
	name := fmt.Sprintf("%s-relay", c.ServerName)

	return &Service{
		ServiceName:   name,
		ContainerName: name,
		HostName:      name,
		Port:          make(map[string]int),
		Secret: map[string]string{
			"ROUTES":  network.Subnet.String(),
			"AUTHKEY": c.Relay.AuthKey,
		},
		Network:     network,
		IPv4Address: net.ParseIP(c.Relay.DockerIP).To4(),
	}, nil
}

func (c *ConfigFile) network() (*Network, error) {
	_, subnet, err := net.ParseCIDR(c.Network.Subnet)
	if err != nil {
		return nil, fmt.Errorf("failed to parse subnet %v", err)
	}

	gw := net.ParseIP(c.Network.Gateway)

	return &Network{
		Name:    fmt.Sprintf("%s-net", c.ServerName),
		Subnet:  subnet,
		Gateway: gw.To4(),
	}, nil
}

func volumePath(directory string, name string) (string, error) {
	dir, err := filepath.Abs(directory)
	if err != nil {
		return "", fmt.Errorf("failed to parse data directory %v", err)
	}

	return filepath.Join(dir, name), nil
}
