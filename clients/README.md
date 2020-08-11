# tb3-ros clients

## VPN Client

To connect to the Tailscale network as robot

```bash
chmod +x pi_connect.sh

# Run the script with the tailscale authkey
sudo ./pi_connect.sh tskey-123abc456

# On successful connect, you should see this
    Connected. IP address: 100.xx.xxx.xxx
```
