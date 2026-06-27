# Hardware Setup

## Parts

- Raspberry Pi 5, 8 GB
- Raspberry Pi Camera Module v2
- Slamtec RPLIDAR A1M8
- MicroSD or NVMe with Raspberry Pi OS
- Power supply rated for Raspberry Pi 5

## First Boot and SSH

This path gets you from an unpowered Pi to a remote shell from your laptop.

1. Flash Raspberry Pi OS with Raspberry Pi Imager.
2. In Imager's OS customization screen, set:
   - Hostname: `lidar-pi`
   - Username and password: choose your own; do not rely on a default `pi` user.
   - Wi-Fi SSID and password if you are not using Ethernet.
   - SSH: enabled, password authentication is fine for first bring-up.
3. Write the image to the microSD card or NVMe drive.
4. With the Pi powered off, connect the Camera Module v2 ribbon cable.
5. Insert the boot media, connect Ethernet if using it, then plug in the Pi 5 power supply.
6. Wait 60 to 90 seconds for the first boot to resize the filesystem and join the network.
7. From Windows PowerShell, try SSH by hostname:

```powershell
ssh <your-username>@lidar-pi.local
```

8. If `.local` does not resolve, find the Pi IP address from your router's device list, then connect by IP:

```powershell
ssh <your-username>@<pi-ip-address>
```

9. Once logged in, run a basic system check:

```bash
hostname -I
uname -a
sudo apt update
```

10. Install the project prerequisites:

```bash
sudo apt install -y git python3-venv python3-picamera2
```

11. Get the project onto the Pi. If you have pushed the repo to GitHub:

```bash
git clone <your-repo-url> lidar-room-mapper
cd lidar-room-mapper
```

If you have not pushed it yet, use `scp` from your laptop or create the remote repo first. The project is easier to demo once it has a GitHub URL.

12. Create the Python environment on the Pi:

```bash
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[hardware]"
```

Use `--system-site-packages` so the venv can import `picamera2`, which Raspberry Pi OS installs through apt. If you already created `.venv` without that flag and camera mode cannot import Picamera2, recreate the venv:

```bash
deactivate
rm -rf .venv
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[hardware]"
```

13. Confirm the package runs before touching hardware:

```bash
python -m lidar_room_mapper scan-once --source sim
```

## Camera

1. Connect the Camera Module v2 to the Raspberry Pi 5 camera connector using the correct ribbon cable.
2. Boot Raspberry Pi OS.
3. Confirm libcamera sees the sensor:

```bash
rpicam-hello --list-cameras
```

4. Install Picamera2 if it is not already present:

```bash
sudo apt update
sudo apt install -y python3-picamera2
```

## RPLIDAR

1. Connect the RPLIDAR A1M8 USB adapter to the Pi.
2. Check the device path:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

3. Allow your current user to access serial devices:

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

4. Run a simulated smoke test first:

```bash
python -m lidar_room_mapper scan-once --source sim
```

5. Run the live sensor:

```bash
python -m lidar_room_mapper scan-once --source rplidar --port /dev/ttyUSB0
```

`scan-once` starts the motor, waits for one LiDAR revolution, prints map statistics, then stops the motor as it exits. If you want the motor to keep spinning while you inspect the map, run the dashboard command below.

## Dashboard

```bash
python -m lidar_room_mapper serve --source rplidar --port /dev/ttyUSB0 --host 0.0.0.0 --camera
```

Open `http://<pi-ip-address>:8000` from a laptop on the same network.

## Bring-Up Order

Use this order when something is not working. It isolates failures quickly:

1. `python -m lidar_room_mapper scan-once --source sim`
2. `rpicam-hello --list-cameras`
3. `ls -l /dev/ttyUSB* /dev/ttyACM*`
4. `python -m lidar_room_mapper scan-once --source rplidar --port /dev/ttyUSB0`
5. `python -m lidar_room_mapper serve --source rplidar --port /dev/ttyUSB0 --host 0.0.0.0 --camera`
6. Open `http://<pi-ip-address>:8000` from your laptop.

## RPLIDAR Troubleshooting

If the LiDAR stops spinning when a command starts, update to the latest driver and reinstall the package:

```bash
git pull
. .venv/bin/activate
python -m pip install -e ".[hardware]"
```

Then try again:

```bash
python -m lidar_room_mapper scan-once --source rplidar --port /dev/ttyUSB0
```

If it still does not scan:

1. Confirm the USB adapter is seated and the RPLIDAR has enough power.
2. Confirm the active port with `ls -l /dev/ttyUSB* /dev/ttyACM*`.
3. Re-run `sudo usermod -aG dialout $USER`, reboot, and SSH back in.
4. Try the dashboard command; it keeps the scan session open instead of stopping after one revolution.

## Export A Recorded Map

After recording `captures/first_room.jsonl`, export a persistent map:

```bash
python -m lidar_room_mapper export-map --source replay --input captures/first_room.jsonl --output artifacts/first_room --scans 200
```

The command writes:

- `artifacts/first_room.png`
- `artifacts/first_room.pgm`
- `artifacts/first_room.yaml`

To inspect whether a moving recording has enough structure for pose-aware mapping:

```bash
python -m lidar_room_mapper scan-match --source replay --input captures/first_room.jsonl --scans 20
```

To export using scan-matched poses:

```bash
python -m lidar_room_mapper export-map --source replay --input captures/first_room.jsonl --output artifacts/first_room_scanmatched --scans 200 --pose-mode scan-match
```

## Primary References

- Raspberry Pi Picamera2 manual: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
- Raspberry Pi camera software docs: https://www.raspberrypi.com/documentation/computers/camera_software.html
- Slamtec support downloads: https://www.slamtec.com/en/Support
- Slamtec RPLIDAR SDK: https://github.com/Slamtec/rplidar_sdk
