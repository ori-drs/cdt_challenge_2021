# cdt_challenge_2021
CDT Challenge 2021: Autonomous exploration with wheeled robots

**Contents**
- [cdt_challenge_2021](#cdt_challenge_2021)
- [Preliminaries](#preliminaries)
  - [Virtual machine setup](#virtual-machine-setup)
  - [SSH keys and GitHub](#ssh-keys-and-github)
  - [Git installation](#git-installation)
  - [Clone this repository](#clone-this-repository)
- [Basic software installation](#basic-software-installation)
  - [Normal installation](#normal-installation)
  - [Step-by-step](#step-by-step)
  - [Updating the software](#updating-the-software)
    - [Updating this package](#updating-this-package)
    - [Updating the system packages](#updating-the-system-packages)
    - [Updating .bashrc](#updating-bashrc)
    - [Fixing elevation mapping](#fixing-elevation-mapping)
- [Running the code](#running-the-code)

# Preliminaries
## Virtual machine setup
*(If you are using a laptop with Ubuntu 18.04, you can skip this step)*

- Download [VMWare Player](https://www.vmware.com/uk/products/workstation-player.html) (recommended choice) or [VirtualBox](https://www.virtualbox.org/wiki/Downloads).
  - The VMWare file is a `.bundle` file. You need to give it execution permission using `chmod +x VMware-Player-16.1.0-17198959.x86_64.bundle`
  - Then, you can simply run the file with `sudo ./VMware-Player-16.1.0-17198959.x86_64.bundle`
- Get an image of [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- Install Ubuntu 18.04 in the virtual machine following these instructions: [VMWare](https://linuxhint.com/install_ubuntu_vmware_workstation/), [VirtualBox](https://www.freecodecamp.org/news/how-to-install-ubuntu-with-oracle-virtualbox/)
- When installing Ubuntu, use the user `cdt2021` and password `cdt` for simplicity
- In order to use fullscreen, you need to install **VMware tools**
  - Start up the CDT virtual machine
  - Log in to the `cdt2021` user
  - Go to the menu bar and click on  *Virtual Machine* -> *Install VMware tools*
  - A cd drive should appear on your desk called *VM Ware tools*
  - Inside there should be a `tar.gz` file. Uncompress it in the desktop
  - Open a terminal, go to the folder you just uncompressed, and run `sudo ./vmware-install.pl`. Follow the instructions to install the tools (spoiler alert, are **a lot**)
  - You should be able to resize the screen as expected after it finishes

Once you're done with the installation, follow the instructions below to setup the machine.

## SSH keys and GitHub
You need to generate a SSH key and added it to your Github account. **This applies to both laptops and virtual machines**, since it's required to clone the repositories automatically.

- [Generate the key](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
- [Add it to your account](https://docs.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account)

## Git installation
Since you need to clone this repository in your machine, you'll need git:
```sh
sudo apt install git
```

## Clone this repository
In your home directory, run the following:
```sh
mkdir git
cd git
git clone git@github.com:ori-drs/cdt_challenge_2021.git
```

# Basic software installation
## Normal installation
Run the following in the `git` folder:

```sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/install.sh
```

## Step-by-step
If the previous script fails at some point, it will indicate which step failed.
You can run each step independently as:

```sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-1-build-environment.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-2-install-ros.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-3-cdt-packages.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-4-update-submodules.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-5-init-workspace.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-6-build.sh
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-7-edit-bashrc.sh

```

## Updating the software
*If you are doing a fresh installation, you can skip this step.*

The instructions below are only relevant if you cloned the repository long time ago.

### Updating this package

```sh
cd ~/git/cdt_challenge_2021
git pull
cd ~/catkin_ws
catkin build
```

### Updating the system packages

```sh
cd ~/git
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-3-cdt-packages.sh
```

### Updating .bashrc
```sh
cd ~/git
./cdt_challenge_2021/cdt_superbuild/build_scripts/setup-7-edit-bashrc.sh
source ~/.bashrc
```

### Fixing elevation mapping
Crashes in the elevation were due to issues in the lidar plugins used in Gazebo. This can be fixed by compiling the lidar plugin:

Clone the `velodyne_simulator` package into your `git` folder

```sh
git clone --branch 1.0.12 https://bitbucket.org/DataspeedInc/velodyne_simulator.git
```

Then go to your `src` folder in your `catkin_ws` folder and symlink it:

```sh
roscd $CATKIN_WORKSPACE
cd src
ln -s ../../git/velodyne_simulator .
```

Build the code:
```sh
catkin build velodyne_gazebo_plugins
```

Add the following line to your `.bashrc` file:

```sh
export GAZEBO_PLUGIN_PATH=$CATKIN_WORKSPACE/devel/lib
```

Source `.bashrc` and try to run the `launch_all.launch` again.


# Running the code
Before running everything, source the `setup.bash` file:

```sh
source ~/catkin_ws/devel/setup.bash
```

To launch everything (simulation and basic stack), you must run:

```sh
roslaunch jackal_runtime_cdt launch_all.launch
```

This will launch Gazebo (without the GUI), Rviz with a basic visualization, and a joystick interface. You should be able to see the lidar scans as well as the lidar elevation map. If you drive the robot with the joystick, the elevation map should be filled in as you go.