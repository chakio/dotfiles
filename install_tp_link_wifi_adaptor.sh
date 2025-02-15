sudo apt-get update && sudo apt-get install -y dkms build-essential linux-headers-generic
# delete the folder if it already exists
sudo rm -rf rtl8812au
git clone https://github.com/aircrack-ng/rtl8812au.git 
cd rtl8812au 
git checkout 63cf0b4
sudo make dkms_remove
sudo make dkms_install
sudo rm -rf rtl8812au
