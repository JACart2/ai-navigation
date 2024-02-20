PKG="./src/ai-navigation/motor_control"

sudo apt remove brltty
sudo cp "${PKG}/resource/99-usb-serial.rules" "/etc/udev/rules.d"

pip install -r ${PKG}/resource/requirements.txt

colcon build --symlink-install
source install/setup.bash