This program is able to decode ITU G.9959 (z-wave) frames using a RTL-SDR dongle

The QI sample output from the rtl_sdr program should be piped into the executable.
the program will pipe the decoded frames to wireshark.

G.9959 frames are FSK/NRZ and FSK/Manchester encoded. Supported bitrates are 
9.6kbps Manchester / 40kbps NRZ / 100kbps NRZ

### EU

    rtl_sdr -f 868420000 -s 2048000 -g 25  - |./rtl_zwave

(this is a fork!)