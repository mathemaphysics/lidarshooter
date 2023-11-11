# Welcome to LiDARShooter

To use the convenient `devcontainer.json` setup for VSCode you'll want to make sure you have Docker or Docker Desktop installed, depending on which OS you're on. Also make sure that the `Dev Container` extension is installed in your VSCode installation.

Before you open and build the devcontainer do the following. If you're on Windows, you'll need to open `.devcontainer/devcontainer.json` and make the following changes: Comments out `remoteEnv`'s `DISPLAY` and `XAUTHORITY` variables and also comment out the last two `source=` lines in the `mounts` section. When you're done with this it should look like this:

```json
	"remoteEnv": {
		//"DISPLAY": "${localEnv:DISPLAY}",
		//"XAUTHORITY": "/home/devuser/.Xauthority",
		"QT_AUTO_SCREEN_SCALE_FACTOR": "1"
	},

	// Maps the .ssh/ folder from your ${HOME} into /home/devuser/.ssh
	"mounts": [
		"source=${localEnv:HOME}${localEnv:USERPROFILE}/.ssh,target=/home/devuser/.ssh,type=bind,consistency=cached",
		//"source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind",
		//"source=${localEnv:XAUTHORITY},target=/home/devuser/.Xauthority,type=bind"
	]
}
```

In order to view the Qt application from inside the devcontainer you'll want to have an X11 server installed for Windows. If you have the latest WSL2 installed then you will likely have one already installed and running by default, i.e. `Xwayland`. You might need to adjust the `DISPLAY` port either way. I believe `:0.0` will work for the `Xwayland` situation. However, if you don't have the latest WSL2 and can't see `Xwayland` running on your GPU (if you have an NVIDIA GPU) by running `nvidia-smi` from a WSL2 terminal, then you'll need to install an X11 server. I prefer MobaXterm for ease of install and the fact that it tells you what address and port its X server is running on when you mouse over the "X" in the upper right corner of the open MobaXterm application. Installing it and setting running `export DISPLAY=[the address that MobaXterm says]` in the bash terminal inside your running devcontainer on Windows. You can save this `DISPLAY` setting in your `devcontainer.json` file in the `remoteEnv` section so you don't have to set it for any additional terminal you open inside the devcontainer.

The default `.devcontainer/devcontainer.json` in the `main` and `dev` branches should work just fine for Ubuntu or Fedora and related OSes.

For MacOS the default configuration may work but may not.

The `XAUTHORITY` etc. changes are needed to make it possible to open the Qt application.

To open in the devcontainer with VSCode, run `Ctrl+Shift+P` and find "Open Folder in Container". Give the top level `lidarshooter` folder as the folder to open. This will run for a while to build the container. When it's done proceed with the build.

# Caveat

If you don't have a `.ssh` folder inside your home directory (on any OS, Windows, Linux, or MacOS), then you'll also want to comment out the first `source=` line in the `mounts` section of `.devcontainer/devcontainer.json` to look like this:

```json
	"mounts": [
		//"source=${localEnv:HOME}${localEnv:USERPROFILE}/.ssh,target=/home/devuser/.ssh,type=bind,consistency=cached",
	]
```

# Building the Source

Build with the following:

```sh
cd ros_ws && catkin_make
```

The default build will use CPU-only raytracing. If you want to use the NVIDIA OptiX raytracer backend then you'll need to copy the Optix 7.6 SDK unzipped into the root folder, i.e. `lidarshooter` at the cloned top level. Specifically it should be extracted to show

```sh
NVIDIA-OptiX-SDK-7.6.0-linux64-x86_64
```

where the folder shown above is inside the directory into which you cloned the repository.

# Starting Up

In order to start up the main program you'll need to source the ROS setup file after the build.

```sh
source ros_ws/devel/setup.bash
roscore &
rosrun lidarshooter LiDARShooterGUI
```

I've been meaning to add an additional container which just runs `roscore` so you don't have to remember to start it up, but I haven't gotten around to it.
