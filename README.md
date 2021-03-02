# cdt_challenge_2021
CDT Challenge 2021: Autonomous exploration with wheeled robots

# Preliminaries
## Virtual machine setup
*(If you are using a laptop with Ubuntu 18.04, you can skip this step)*

- Download [VMWare Player](https://www.vmware.com/uk/products/workstation-player.html) (recommended choice) or [VirtualBox](https://www.virtualbox.org/wiki/Downloads).
- Get an image of [Ubuntu 18.04](https://releases.ubuntu.com/18.04/)
- Install Ubuntu 18.04 in the virtual machine following these instructions: [VMWare](https://linuxhint.com/install_ubuntu_vmware_workstation/), [VirtualBox](https://www.freecodecamp.org/news/how-to-install-ubuntu-with-oracle-virtualbox/)
- When installing Ubuntu, use the user `cdt2021` and password `cdt` for simplicity
Once you're done with Ubuntu, follow the instructions below to setup the machine.

## SSH keys and GitHub
You need to generate a SSH key and added it to your Github account. **This applies to both laptops and virtual machines**, since it's required to clone the repositories automatically.

- [Generate the key](https://docs.github.com/en/github/authenticating-to-github/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)
- [Add it to your account](https://docs.github.com/en/github/authenticating-to-github/adding-a-new-ssh-key-to-your-github-account)

## Basic software on your machine
Since you need to clone this repository in your machine, you'll need git:
```bash
sudo apt install git
```

## Clone the repository
In your home directory, run the following:
```bash
mkdir git
cd git
git clone git@github.com:ori-drs/cdt_challenge_2021.git
```

# Basic software installation
## Normal installation
Run the following in the `git` folder:

```bash
./cdt_superbuild/build_scripts/install.sh
```

## Step-by-step
If the previous script fails at some point, it will indicate which step failed.
You can run each step independently as:

``` bash
./cdt_superbuild/build_scripts/setup-1-build-environment.sh
./cdt_superbuild/build_scripts/setup-2-install-ros.sh
./cdt_superbuild/build_scripts/setup-3-cdt-packages.sh
./cdt_superbuild/build_scripts/setup-4-update-submodules.sh
./cdt_superbuild/build_scripts/setup-5-init-workspace.sh
./cdt_superbuild/build_scripts/setup-6-build.sh
./cdt_superbuild/build_scripts/setup-7-edit-bashrc.sh

```

# Updating the software
This instructions are relevant if you cloned the repository long time ago.

## Updating this package

```sh
cd ~/git/cdt_challenge_2021
git pull
cd ~/catkin_ws
catkin build
```

## Updating packages

```sh
cd ~/git
./cdt_superbuild/build_scripts/setup-3-cdt-packages.sh
```

## Updating .bashrc
```sh
cd ~/git
./cdt_superbuild/build_scripts/setup-7-edit-bashrc.sh
source ~/.bashrc
```
