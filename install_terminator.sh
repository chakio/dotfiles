sudo apt-get install terminator -y
mkdir ~/.config/terminator
ln -fs ~/dotfiles/terminator_config  ~/.config/terminator/config
echo "alias rosterm='terminator -l ros&exit'" >> ~/.bashrc