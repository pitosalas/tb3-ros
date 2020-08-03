# serverctl

A cli utility for creating a `tb3-ros` server.

## Requirement

- `docker ce`
- `docker-compose`
- Domain name with wildcard A record, ie. `A  *.roslab.example.com  123.123.123.123`
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
  names:
    - "mars"                  # Names of the desktops
    - "saturn"
    - "jupiter"
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

Check `build/list.md` for all the credentials of every desktop and traefik interface.

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
- `build/list.md`

`build/list.md` contains all the credentials of every desktop and traefik interface.
