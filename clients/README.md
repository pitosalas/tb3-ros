# Private Networking

## Client (raspberry pi)

Supported OS: `Raspbian 10, Ubuntu 18.04, Ubuntu 20.04`

To connect to the Tailscale network as robot

```bash
chmod +x pi_connect.sh

# Run the script with the tailscale authkey
sudo ./pi_connect.sh tskey-123abc456

# On successful connect, you should see this
Connected. IP address: 100.xx.xxx.xxx
```

Once the robot is successfully connected to the network, you can try to reach it via cloud desktop

```bash
ssh pi@100.xx.xxx.xxx
```

On subsequence restarts/disconnects, you can connect with the script

```bash
sudo ./pi_connect.sh tskey-123abc456
```

## Removing Tailscale

```bash
sudo apt-get remove -y tailscale
sudo rm -rf /var/lib/tailscale/tailscaled.state

# Reboot right after!
sudo reboot
```
