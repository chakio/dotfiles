#!/bin/sh

type docker > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "docker already installed"
else    
    sudo apt-get update -y
    sudo apt-get install -y \
        apt-transport-https \
        ca-certificates \
        curl \
        gnupg-agent \
        software-properties-common
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    sudo apt-key fingerprint 0EBFCD88
    sudo add-apt-repository \
    "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"
    sudo apt-get update -y
    sudo apt-get install docker-ce docker-ce-cli containerd.io -y
    echo "docker installed"

    sudo gpasswd -a $(whoami) docker
    sudo chgrp docker /var/run/docker.sock
    sudo service docker restart
fi
