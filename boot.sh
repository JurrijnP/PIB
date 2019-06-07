#!/bin/sh -e
sleep 30;echo "[ AUTO ] Bluetooth set to discoverable."
hciconfig hci0 piscan;echo "[ AUTO ] PulseAudio started."
sudo pulseaudio --start;echo "[ INFO ] Bluetooth pairing ready.";exit 0