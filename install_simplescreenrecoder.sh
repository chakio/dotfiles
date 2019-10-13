simplescreenrecorder --version&> /dev/null
if [ $? -ne 0 ]; then
    sudo add-apt-repository ppa:maarten-baert/simplescreenrecorder -y
    sudo apt-get update -y
    sudo apt-get install simplescreenrecorder -y
    echo "Simple Screen Recorder installed"
else    
    echo "Simple Screen Recorder already installed"
fi