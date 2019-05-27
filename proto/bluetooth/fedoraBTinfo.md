# this file contains snippets of settings for ESP32 BT serial comm with fedora linux distro



# Should work
sudo rfcomm bind rfcomm0 30:AE:A4:FF:1B:A2 || chmod 666 /dev/rfcomm0


# other snippets..

30:AE:A4:FE:68:02

30AEA4FE6802


sudo hcitool info 30:AE:A4:FE:68:02

sudo rfcomm bind rfcom0 30:AE:A4:FE:68:02

 sudo rfcomm connect 1 30:AE:A4:FE:68:02 2

sudo chmod 666 /dev/rfcomm0


sudo hcitool info 30:AE:A4:FF:1B:A2
sudo rfcomm bind rfcom1 30:AE:A4:FF:1B:A2
sudo rfcomm connect 1 30:AE:A4:FF:1B:A2 2 || chmod 666 /dev/rfcomm1
sudo chmod 666 /dev/rfcomm1


sudo rfcomm bind rfcomm1 30:AE:A4:FF:1B:A2 || chmod 666 /dev/rfcomm1


sudo rfcomm unbind rfcom2 30:AE:A4:FF:1B:A2

sudo rfcomm bind rfcom2 30:AE:A4:FF:1B:A2
