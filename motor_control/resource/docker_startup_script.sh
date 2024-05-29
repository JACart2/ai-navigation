PKG="./src/ai-navigation/motor_control"

# brltty is a package that conflicts with the driver that arudino uses for its usb connection
sudo apt remove brltty

# sets up a rule file that automatically sets the arduino to port "ttyUSB9" everytime
sudo cp "${PKG}/resource/99-usb-serial.rules" "/etc/udev/rules.d"

# installs all required packages
pip install -r ${PKG}/resource/requirements.txt

# builds system
colcon build --symlink-install
. install/setup.sh
