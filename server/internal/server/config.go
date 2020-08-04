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
	Server struct {
		Name      string `yaml:"name"`
		Domain    string `yaml:"domain"`
		BuildPath string `yaml:"build_path"`
	}
	Desktop struct {
		Names    []string `yaml:",flow"`
		Security struct {
			RandomPassword bool   `yaml:"random_password"`
			Password       string `yaml:",omitempty"`
		}
		Data struct {
			Directory string `yaml:"directory"`
		}
	}
	Proxy struct {
		HTTPPort int `yaml:"http_port"`
		TLSPort  int `yaml:"tls_port"`
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
		return nil, fmt.Errorf("unable to read config file: %v", err)
	}

	err = yaml.Unmarshal(f, &configFile)
	if err != nil {
		return nil, fmt.Errorf("failed to parse config file: %v", err)
	}

	return configFile.Services()
}

func (c *ConfigFile) Services() (*Services, error) {
	network, err := c.network()
	if err != nil {
		return nil, fmt.Errorf("failed to parse network: %v", err)
	}

	proxy, err := c.proxy(network)
	if err != nil {
		return nil, fmt.Errorf("failed to parse proxy: %v", err)
	}

	relay, err := c.relay(network)
	if err != nil {
		return nil, fmt.Errorf("failed to parse relay: %v", err)
	}

	desktopList, err := c.desktopList(network)
	if err != nil {
		return nil, fmt.Errorf("failed to parse desktop: %v", err)
	}

	return &Services{
		DesktopList: desktopList,
		Proxy:       proxy,
		Relay:       relay,
		Network:     network,
		Config:      c,
	}, nil
}

func (c *ConfigFile) desktopList(network *Network) ([]*Service, error) {
	desktopList := make([]*Service, 0, len(c.Desktop.Names))

	for _, n := range c.Desktop.Names {
		name := fmt.Sprintf("%s-%s", c.Server.Name, n)

		var password string
		if c.Desktop.Security.RandomPassword {
			password = faker.Password()
		} else {
			password = c.Desktop.Security.Password
		}

		vName, vPath, err := persistentVolume(c.Desktop.Data.Directory, name)
		if err != nil {
			return nil, fmt.Errorf("failed to create desktop list: %v", err)
		}

		s := &Service{
			Name:          n,
			ServiceName:   name,
			ContainerName: name,
			HostName:      name,
			Secrets: map[string]string{
				"GW_IP":    network.Gateway.String(),
				"RELAY_IP": net.ParseIP(c.Relay.DockerIP).To4().String(),
				"PASSWORD": password,
				"DOMAIN":   c.Server.Domain,
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
	name := fmt.Sprintf("%s-relay", c.Server.Name)

	vName, vPath, err := persistentVolume(c.Desktop.Data.Directory, name)
	if err != nil {
		return nil, fmt.Errorf("failed to create relay: %v", err)
	}

	return &Service{
		ServiceName:   name,
		ContainerName: name,
		HostName:      name,
		Port:          make(map[string]int),
		Secrets: map[string]string{
			"ROUTES":  network.Subnet.String(),
			"AUTHKEY": c.Relay.AuthKey,
		},
		Network:     network,
		IPv4Address: net.ParseIP(c.Relay.DockerIP).To4(),
		Volume: &Volume{
			Name: vName,
			Path: vPath,
		},
	}, nil
}

func (c *ConfigFile) proxy(network *Network) (*Service, error) {
	name := fmt.Sprintf("%s-proxy", c.Server.Name)

	return &Service{
		ServiceName:   name,
		ContainerName: name,
		HostName:      name,
		Port: map[string]int{
			"http": c.Proxy.HTTPPort,
			"tls":  c.Proxy.TLSPort,
		},
		Secrets: map[string]string{
			"DOMAIN": c.Server.Domain,
		},
		Network: network,
	}, nil
}

func (c *ConfigFile) network() (*Network, error) {
	_, subnet, err := net.ParseCIDR(c.Network.Subnet)
	if err != nil {
		return nil, fmt.Errorf("failed to parse subnet: %v", err)
	}

	gw := net.ParseIP(c.Network.Gateway)

	return &Network{
		Name:    fmt.Sprintf("%s-net", c.Server.Name),
		Subnet:  subnet,
		Gateway: gw.To4(),
	}, nil
}

func persistentVolume(directory string, name string) (string, string, error) {
	vName := fmt.Sprintf("rospersistent-%s", name)
	vPath, err := volumePath(directory, vName)
	if err != nil {
		return "", "", fmt.Errorf("failed to create persistent volume: %v", err)
	}

	return vName, vPath, nil
}

func volumePath(directory string, name string) (string, error) {
	dir, err := filepath.Abs(directory)
	if err != nil {
		return "", fmt.Errorf("invalid directory: %v", err)
	}

	return filepath.Join(dir, name), nil
}
