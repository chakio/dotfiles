#!/bin/bash
echo "source $HOME/dotfiles/.aliases">> $HOME/.bashrc
echo "source $HOME/dotfiles/.ros_aliases">> $HOME/.bashrc
source $HOME/.bashrc