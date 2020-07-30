#!/bin/bash

infof=`tput setaf 6`
errorf=`tput setaf 1`
donef=`tput setaf 2`
reset=`tput sgr0`

if ! command -v tailscale &> /dev/null
then
    echo "${errorf}  Tailscale not found${reset}"
    echo "${infof}  Installing Tailscale client${reset}"
    echo "${infof}  Adding Tailscale repo${reset}"
    sudo apt-get install -y apt-transport-https
    curl https://pkgs.tailscale.com/stable/raspbian/buster.gpg | sudo apt-key add -
    curl https://pkgs.tailscale.com/stable/raspbian/buster.list | sudo tee /etc/apt/sources.list.d/tailscale.list

    echo "${infof}  Installing Tailscale${reset}"
    sudo apt-get update -y && apt-get install -y tailscale
    echo
fi

if [ "$1" != "" ]; then
    echo -ne "${infof}  Connecting...\r${reset}"
    sudo tailscale up --authkey=$1
    echo "${donef}  Connected. IP address: $(ifconfig tailscale0 | grep 'inet ' | awk '{print $2}')${reset}"
else
    echo "${errorf}  ERROR: Missing Authkey${reset}"
    echo "${errorf}  Please run again with: ./pi_connect.sh tskey-123abc...${reset}"
fi
