# tb3-ros clients

## VPN Client (raspberry pi)

Supported OS: `Debian 10, Ubuntu 18.04, Ubuntu 20.04`

To connect to the Tailscale network as robot

```bash
chmod +x pi_connect.sh

# Run the script with the tailscale authkey
sudo ./pi_connect.sh tskey-123abc456

# On successful connect, you should see this
Connected. IP address: 100.xx.xxx.xxx
```

On subsequence restarts/disconnects, you can connect with the script

```bash
sudo ./pi_connect.sh tskey-123abc456
```
