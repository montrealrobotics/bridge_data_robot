# Bridge Data Robot

Code for controlling Trossen WidowX robot arms.

- `widowx_envs`: contains the `widowx_envs` Python package with all of the WidowX controller code.
- `docker-compose.yml`: contains all of the docker-compose services that will be used to run the robot.

## Setup

The robot software (ROS 2 Jazzy, the Interbotix drivers and this repo) runs inside Docker, so the host
only needs a few things installed.

### 1. Install the host dependencies

Run `./host_install.sh` on the robot comntrol computer (Ubuntu 24.04). It asks what to install:

| Prompt | Needed? | What it does |
|---|---|---|
| Install WidowX drivers? | **yes** | Installs the Interbotix udev rule on the host: creates `/dev/ttyDXL` for the arm and sets the USB latency to 1 ms. udev rules only work on the host, not inside Docker. |
| Install Docker? | **yes** | Answer yes to "non-root user" so you can run `docker` without `sudo` (log out and back in afterwards). |
| Install nvidia-docker? | only for GPU inference here | The robot container doesn't need it. Only if this computer also runs policy inference in Docker on an NVIDIA GPU (see Evaluating policies). |
| Install docker-compose? | **yes** | Used to build and run the containers. |
| Install ROS on the host? | **no** (optional) | See below. |

**ROS on the host is not required.** ROS is installed in the container. Installing it on the host only
lets you run ROS tools (`ros2 topic list`, `ros2 topic hz`, `rviz2`) directly on this machine instead of
through `docker compose exec` (the containers use host networking, so host tools see their topics). It
installs the same distro as the containers (Jazzy on 24.04, Humble on 22.04) but is a large install
and can take a while, so skip it unless you want those tools.

### 2. Build and start the robot container

The ROS service that talks to the robot is the `robonet` entry in `docker-compose.yml`. It uses the
`robonet-base` image, built from the `Dockerfile` in this directory. To build and run it:

```bash
# first generate the usb config file
./generate_usb_config.sh

# build and run the robonet service
USB_CONNECTOR_CHART=$(pwd)/usb_connector_chart.yml docker compose up --build robonet
```

This builds the `robonet-base` image, which contains all of the ROS dependencies and the Python controller code from the `widowx_envs` directory. The USB connector chart is required to start the camera stream. You can get the USB device IDs by running `v4l2-ctl --list-devices`, `./generate_usb_config.sh` automatically generates the config file for you.

Once this is running, you can execute commands in the running container like so:

```bash
docker compose exec robonet bash -lic "go_sleep"
```

Explanation:
- `docker compose exec`: execute a command in a running container
- `robonet`: the service name (as specified in `docker-compose.yml`)
- `bash`: the executable to run inside the container
- `-l`: tells bash to open a login shell, sourcing `~/.bashrc` inside the container, which is required to set up a few ROS things and the correct Python virtual environment
- `-i`: makes the shell interactive, in case you want to run an interactive command (like `python`)
- `-c`: tells bash to execute the next argument
- `go_sleep`: the string to be executed by bash; in this case, it's a utility script that is built in to the `robonet-base` image that moves the arm to the sleep position

If you really want to, you can also attach a bash shell interactively using `docker compose exec robonet bash`.

### Using RealSense cameras

RealSense cameras use their own ROS driver instead of the webcam streamers. To use one, set these in the
`robonet` service of `docker-compose.yml` and restart the container:

```yaml
- USE_REALSENSE=true
```

### Camera configuration

With the container running, open the camera selector (from an `ssh -X` session or a desktop, as it opens
a window):

```bash
docker compose exec robonet bash -lic "python3 ~/widowx_envs/widowx_envs/select_cameras.py"
```

It shows a live thumbnail of every camera topic that is publishing. Click the thumbnails of all camera feeds you want to use, press `s` to save,
`q` to quit. The selection is saved to `widowx_envs/cameras.yaml` and used by data collection, teleop
and eval. You only need to run it again when your cameras change.

## Data collection

First, make sure the `robonet-base` container is running using the above command. Then, run the following commands:

Data is saved to `./trainingdata` (mounted in the container at `/home/robonet/trainingdata`).

```bash
# access the container
docker compose exec robonet bash

# start the data collection script
python widowx_envs/widowx_envs/run_data_collection.py widowx_envs/experiments/bridge_data_v2/conf.py
```

You can specify a different directory to save the data in `docker-compose.yml`.

At this point, the data_collection script will start initializing, and then throw the error:
```bash
Device not found. Make sure that device is running and is connected over USB
Run `adb devices` to verify that the device is visible.
```

This is expected, as our data collection requires the use of a Meta Quest VR headset to control the widowx arm. Turn on and connect the VR headset to the computer via USB. Make sure USB debugging is enabled and answer "Yes/Agree" to any prompts that appear in the headset. 

## Evaluating policies

Policies run on an inference machine that controls the robot over the network (server-client): the robot
control computer runs a server that receives actions, and the policy runs as a client that sends them. The
inference machine does **not** need ROS and can be a separate GPU machine, or the robot computer itself.

**1. Start the server on the robot computer:**

```bash
docker compose exec robonet bash -lic "widowx_env_service --server"
```

**2. Set up the inference machine** with our policy code,
[montrealrobotics/bridge_data_v2](https://github.com/montrealrobotics/bridge_data_v2) (branch `ubuntu_24`):
follow its README's *Environment* section (Docker or a uv Python environment), then its *Evaluation*
section to run a policy with `experiments/eval.py --ip <ROBOT_IP>`.

**3. Check the connection** from the inference machine (the robot executes a few predefined movements):

```bash
widowx_env_service --client --ip <ROBOT_IP>
```

`--ip` defaults to `localhost` (inference on the robot computer). The cameras are defined
in (`widowx_envs/cameras.yaml`); override with `--camera_topics` or
`WIDOWX_CAMERA_TOPICS` if needed.

Extra util, to teleoperate the arm remotely with the keyboard:

```bash
python -m widowx_envs.teleop --ip <ROBOT_IP>
```

## Troubleshooting

##### Permission errors

If you run into following errors:

```bash
Traceback (most recent call last):
  File "urllib3/connectionpool.py", line 677, in urlopen
  File "urllib3/connectionpool.py", line 392, in _make_request
  File "http/client.py", line 1277, in request
  File "http/client.py", line 1323, in _send_request
  File "http/client.py", line 1272, in endheaders
  File "http/client.py", line 1032, in _send_output
  File "http/client.py", line 972, in send
  File "docker/transport/unixconn.py", line 43, in connect
PermissionError: [Errno 13] Permission denied
```
that can be fixed by running the following commands and subsequently restarting the PC (the log out and log back in is sometimes not sufficient):

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```
