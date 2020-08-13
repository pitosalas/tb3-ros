# tb3-ros serverctl

A cli utility for creating a `tb3-ros` server.

## Requirement

- `docker ce > 18.09`
- `docker-compose`
- Domain name with wildcard A record pointing the server's IP, ie. `A  *.roslab.example.com  123.123.123.123`
- Tailscale [Authkey](https://login.tailscale.com/admin/authkeys)

## Install

```bash
make docker-build
# For linux
mv bin/linux/serverctl .
# For mac
mv bin/darwin/serverctl .
```

## Configure

Configure the server by editing `config.yaml`

```bash
mv config.yaml.example config.yaml
```

```yaml
# config.yaml
server:
  name: example
  domain: roslab.example.com  # Hosting server's domain
  build_path: ./build         # Path for outputting build files (ie. docker-compose.yaml)
desktop:
  instances:
    - name: mars              # Name of cloud desktop
      owner: Admin            # Name of cloud desktop's owner 
    - name: saturn
    - name: jupiter
  security:
    random_password: false    # Generate random password for each desktop or not
    password: dev@ros
  data:
    directory: ./data         # Path for where the persistent data will be stored
relay:
  authkey: tskey-123abc       # Tailscale Authkey, see https://login.tailscale.com/admin/authkeys
  docker_ip: 172.99.0.255     # Static internal IP of the relay
proxy:
  http_port: 80               # Default port for all http services like novnc, vscode
  tls_port: 443               # HTTPS support
network:
  subnet: 172.99.0.0/16       # Internal subnet
  gateway: 172.99.0.1         # Internal subnet gateway
```

## Use

### Up

`Up` starts the server based on `config.yaml`.

```bash
./serverctl up -c config.yaml
```

Check `build/overview.md` for all the credentials of every cloud desktop and traefik interface.

### Down

`Down` stops the server.

```bash
./serverctl down -c config.yaml
```

### Generate

`Generate` generates the files for starting the server.

```bash
./serverctl generate -c config.yaml
```

2 Files will be generated

- `build/docker-compose.yaml`
- `build/overview.md`

`build/overview.md` contains all the credentials of every cloud desktop and traefik interface.

## Network

The server uses Tailscale to form a private network connecting cloud desktops and robots.

When the server is started with `up`, it creates a relay that connects all cloud desktops to the Tailscale network, allowing them to connect to any robots on the network.

```

desktop-1 \        _ _ _ _ _        / robot-1 100.89.2.122
           \      |         |      /
desktop-2 - - - - |  Relay  | - - - - robot-2 100.99.31.234
           /      |_ _ _ _ _|      \
desktop-3 /      100.64.10.101      \ robot-3 100.86.232.111

```

To connect to any robot, simply do:

```bash
ssh root@100.89.2.122 # IP of robot-1
```
