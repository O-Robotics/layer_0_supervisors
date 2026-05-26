# Robot Web Access Network Defaults

Target operator URL:

- `http://192.168.2.1:8080`

Target static robot Wi-Fi address:

- `192.168.2.1/24`

## Intended host setup

1. Copy [99-amr-sweeper-static.yaml](/mnt/c/home/dev/rob_ws/src/layer_0_supervisors/amr_sweeper_web_server/config/network/99-amr-sweeper-static.yaml) into `/etc/netplan/`.
2. Confirm the robot Wi-Fi interface name is `wlan0`.
3. Apply with `sudo netplan apply`.

## Notes

- The web server binds to `0.0.0.0:8080`; the static IP determines how operators reach it.
- This assumes the robot-managed Wi-Fi LAN uses the `192.168.2.0/24` subnet.
- If the Wi-Fi interface is not `wlan0`, update the netplan file before applying it.
