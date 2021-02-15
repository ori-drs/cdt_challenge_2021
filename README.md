# cdt_challenge_2021
CDT Challenge 2021: Autonomous exploration with wheeled robots

# Preliminaries
## SSH keys and GitHub
You need to have generated a SSH key and add it to your Github account.
This is required to clone the repositories automatically.

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
