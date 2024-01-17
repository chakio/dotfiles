sudo apt-get update && sudo apt-get install -y dkms build-essential linux-headers-generic
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au && sudo make dkms_install
sudo rm -rf rtl8812au
