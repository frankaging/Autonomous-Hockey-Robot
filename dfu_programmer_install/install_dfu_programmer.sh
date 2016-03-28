
#!/bin/sh
echo A dfu-programmer install script.
echo by: Uriah Baalke
sudo echo Installing dfu_programmer
cd libusb-1.0.8
./configure
make
sudo make install
make clean
cd ..
cd libusb-compat-0.1.3
./configure LIBUSB_1_0_CFLAGS=-I/usr/local/include/libusb-1.0 LIBUSB_1_0_LIBS="-L/usr/local/lib -lusb-1.0"
make
sudo make install
make clean
cd ..
cd dfu-programmer
./configure LIBUSB_1_0_CFLAGS=-I/usr/local/include/libusb-1.0 LIBUSB_1_0_LIBS="-L/usr/local/lib -lusb-1.0"
make
sudo make install
make clean
man dfu-programmer