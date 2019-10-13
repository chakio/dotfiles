google-chrome --version&> /dev/null
if [ $? -ne 0 ]; then
    sudo sh -c 'echo "deb http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list' -y
    sudo wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - -y
    sudo apt-get update -y
    sudo apt-get install google-chrome-stable -y
    echo "Google Chromeinstalled"
else    
    echo "Google Chrome already installed"
fi