# serverctl

A cli utility for creating a `tb3-ros` server.

## Requirement

- `docker ce`
- `docker-compose`

## Use

### Config

Configure the server

```bash
mv config.yaml.example config.yaml
```

### Up

`Up` starts the server based on `config.yaml`

```bash
./serverctl up -c config.yaml -d
```

### Down

`Down` stops the server

```bash
./serverctl down -c config.yaml
```

### Generate

`Generate` generates the `docker-compose.yaml`

```bash
./serverctl generate -c config.yaml
```
