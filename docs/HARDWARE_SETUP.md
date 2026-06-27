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
python3 -m venv .venv
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

## Primary References

- Raspberry Pi Picamera2 manual: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
- Raspberry Pi camera software docs: https://www.raspberrypi.com/documentation/computers/camera_software.html
- Slamtec support downloads: https://www.slamtec.com/en/Support
- Slamtec RPLIDAR SDK: https://github.com/Slamtec/rplidar_sdk
