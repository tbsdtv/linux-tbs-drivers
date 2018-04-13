#!/bin/bash

make distclean

echo "TBS drivers set for x64 Linux"
./v4l/tbs-x86_64.sh
#./v4l/tbs-dvbc-x86_64.sh

# Enable some staging drivers
make stagingconfig

echo "TBS drivers building..."
make -j4

echo "TBS drivers installing..."
sudo rm -r -f /lib/modules/$(uname -r)/extra
sudo make install

echo "TBS drivers installation done"
echo "You need to reboot..."
