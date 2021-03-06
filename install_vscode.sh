curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt install apt-transport-https
sudo apt update
sudo apt-get install code


#エディタの設定を共有

if [ -d ~/.config/Code/ ]; then
	rm  -rf ~/.config/Code/User/
else
	mkdir ~/.config/Code/
fi
ln -fs ~/dotfiles/vscode/User/ ~/.config/Code/
#プラグインを共有
if [ -d ~/.vscode/ ]; then
	rm  -rf ~/.vscode/extentions/
else
	mkdir ~/.vscode/
fi

ln -fs ~/dotfiles/vscode/extensions/ ~/.vscode/
