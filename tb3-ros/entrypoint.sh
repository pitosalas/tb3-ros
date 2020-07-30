
#!/bin/bash

if [[ ! -z "${GW_IP}" ]]; then
  echo "setting up routes for tailscale-relay"
  ip route del default
  ip route add 100.64.0.0/10 via $RELAY_IP
  ip route add default via $GW_IP
fi

echo "starting xvfb"
Xvfb :99 -ac -screen 0 "$XVFB_WHD" -nolisten tcp &
Xvfb_pid="$!"

echo "setting up VNC password"
mkdir ~/.vnc
x11vnc -storepasswd $PASSWORD ~/.vnc/passwd

echo "starting the x11 vnc server"
x11vnc -display :99 --loop -noxrecord -rfbauth ~/.vnc/passwd &

echo "checking openGl support"
glxinfo | grep '^direct rendering:'

echo "starting window manager jwm"
jwm -f /entry/.jwmrc &

echo "starting noVNC"
/novnc/noVNC/utils/launch.sh --vnc localhost:5900 &

echo "starting sshd"
/etc/init.d/ssh restart

source /opt/ros/melodic/setup.bash
/code-server/code-server --user-data-dir /workspace --allow-http --password $PASSWORD --auth password
