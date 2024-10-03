# Brake Signal Transmitter Testing with STM32MP157F-DK2
## Objectives
In testing the Brake Signal Transmitter with the MPU STM32MP157F-DK2, we want to make sure a number of this happen:
- Set up Custom Image using Yocto Project on sd card to flash STM32MP157F-DK2
- Read PWM Inputs from Brake Signal Transmitter
- Read String Potentiometer for Stroke
- Receive Communications from Cortex A7 through IPCC OpenAMP to start a specific process in Cortex M4
- Transmit Communications from Cortex M4 through IPCC OpenAMP to acknowledge the start/end of the process and to send data (duty cycle 1, duty cycle 2, frequency 1, frequency 2, stroke, timestamp, pass or fail) to Cortex A7
- Display the data through the Cortex A7 Web Server using Rust and Webassembly for web application
- ## Initial Setup Instructions

### PCB Design
- 
### Yocto Custom Image
- We are making a custom image due to specifications in our project not being concerned with a lot of the packages in the meta-st-openstlinux layer which contains the framework metadata for OpenSTLinux distribution.

To build the yocto image for this board:


**Pre-Requisites**:
- At least 50 gigabytes of free space
- At least 4 gigabytes of RAM
- Update your system
Debian-based
```zsh
sudo apt update
```
or if you are using Arch-based
```zsh
sudo pacman -Syu
```
- Install Dependencies
Debian-based
```zsh
sudo apt install bc build-essential chrpath cpio diffstat gawk git texinfo wget gdisk python3 python3-pip
```
Arch-based
```zsh
sudo pacman -Syu bc base-devel chrpath cpio diffstat gawk git texinfo wget gdisk python3 python-pip
```
- Specifically for the stm32mp1 build process in yocto, you need the development version of the ssl library
Debian-based
```zsh
sudo apt install -y libssl-dev
```
Arch-based
```zsh
sudo pacman -Syu openssl
```

**Downloading Poky**:
- [[Poky]] is the reference distribution consisting of:
	- [[Metadata]] 
	- [[Layers]]
	- [[Board Support Package]]s
	- BitBake
	- etc.
- Make up the yocto project and openembedded
- Generates various images in a good fashion
- Using Poky we add our own layers to create a customized version of our own distribution
- Go to the directory you want to clone the repository to then run
```zsh
git clone git://git.yoctoproject.org/poky.git
```
- You also need to check out the scarthgap branch will be explained in a few steps as this is the branch I have chosen to use
- In the /poky directory
```zsh
git checkout scarthgap
```

**Bitbake**:
- Main tool being used to read in
	- Metadata
	- Recipes
	- Board support packages
- All of the meta folders are our layers consisting of metadata that gets used by the bitbake process

**Branches**
- Make sure branches match
- Dylan is using the Dunfell Codename
- In this implementation, I am using the Scarthgap codename due to its Long Term Support level
	- In layers from other vendors or board support packages, you need to check out the version that lines up with the codename you are using
	- The following command below shows how to checkout a layer with the codename
```zsh
cd meta-openembedded
git checkout scarthgap
```
- Note: After checking out a branch, it is recommended you look at the distribution config file
	- Example in /poky/meta-poky/conf/distro/poky.conf:
		- Look for ``` DISTRO = "poky" ``` and ``` DISTRO_CODENAME = "scarthgap" ``` 



**Board Support Package**:
- Using STM's STMicroelectronics/meta-st-stm32mp board support package
- ST created this BSP to support their various single-board computers, reference board, or evaluation kits
- Import these files into the Yocto Project to supplement the poky reference distrubution and create the custom linux build. 
- In the same directory level as poky run
```zsh
git clone git@github.com:STMicroelectronics/meta-st-stm32mp
```
- Check out the scarthgap branch to use this after cloning into the /poky directory
- In the /meta-st-stm32mp directory run
```zsh
git checkout scarthgap
```
- In the /meta-st-stm32mp/README.md it tells us we find that this layer has dependencies on
	- openembedded-core at the layers level
	- poky - at the layers level
	- meta-openembedded - at the layers level
- In the same directory as the poky layer and meta-st-stm32mp layer run
```zsh
git clone git@github.com:openembedded/openembedded-core
git clone git@github.com:openembedded/meta-openembedded
```
- Have to checkout both of these layers by going into their directory and running
```zsh
git checkout scarthgap
```

**Build Environment**:
- Once back on the meta layer, we want to run the open-embedded initial build environment found in the poky directory
- This will create a build environment and will do things such as add bitbake to our path so we can call it with other tools needed
- It will also create a build directory for you with the default name build, I give mine the name of build-mp1 to specify the build is for the mp1 microprocessing unit
```zsh
source poky/oe-init-build-env build-mp1
```
- Run bitbake-layers to show the layers in this build environment
```zsh
bitbake-layers show-layers
```
- In here, you will see the layers that are currently in the build environment
- We need the layers for the Board Support Package along with it's dependencies in /conf/bblayers.conf:
```conf
# POKY_BBLAYERS_CONF_VERSION is increased each time build/conf/bblayers.conf
# changes incompatibly
POKY_BBLAYERS_CONF_VERSION = "2"

BBPATH = "${TOPDIR}"
BBFILES ?= ""

# First 3 Layers Included
# Added meta-oe and meta-python for BSP dependencies
# Added meta-multimedia, meta-networking, meta-webserver for project purposes
# Added meta-rust-bin and meta-clang for language servers
# Added BSP package
BBLAYERS ?= " \
  /home/danb127/EmbeddedLinux/yocto/poky/meta \
  /home/danb127/EmbeddedLinux/yocto/poky/meta-poky \
  /home/danb127/EmbeddedLinux/yocto/poky/meta-yocto-bsp \
  /home/danb127/EmbeddedLinux/yocto/meta-openembedded/meta-oe \
  /home/danb127/EmbeddedLinux/yocto/meta-openembedded/meta-python \
  /home/danb127/EmbeddedLinux/yocto/meta-openembedded/meta-multimedia \
  /home/danb127/EmbeddedLinux/yocto/meta-openembedded/meta-networking \
  /home/danb127/EmbeddedLinux/yocto/meta-openembedded/meta-webserver \
  /home/danb127/EmbeddedLinux/yocto/meta-rust-bin \
  /home/danb127/EmbeddedLinux/yocto/meta-clang \
  /home/danb127/EmbeddedLinux/yocto/meta-st-stm32mp \
  "
```
- Check your bitbake-layers again to show that you now have included the additional layers

**Machine**:
- We now want to look into the machine we want
- The machine I will be using is the stm32mp15-disco
- You can see what machines are available for the BSP running
```zsh
cd meta-st-stm32mp/conf
cd machine
```
- After picking your machine you want to go back to the meta layers and run 
```zsh
cd build-mp1
nvim conf/local.conf
```
- In this file you will want to edit the machine to the desired machine
```conf
#MACHINE ??= "qemux86-64"
MACHINE = "stm32mp15-disco"
```

**Image Build**
- I selected core-image-minimal to build because we just want a simple boot and don't need anything such as an emulator for graphics.
- With this knowledge we need to run within the build-mp1 environment
```zsh
bitbake core-image-minimal
```

**Output Images**
- When you are done you can find all of the output images by running the command in your build environment
```zsh
ls tmp/deploy/images/stm32mp15-disco 
```

## Boot Process and SD Card
[[ZF Internship - STM32 MPU Boot Chain Overview]]
### CubeMX
Can use git to look into project by installing this repository in your directory
```zsh
git clone git@github.com:danb127/Brake-System-Tester
```
- STM32 CubeMX Initialization
	- 2 Timers for 2 [[Pulse Width Modulation]] Signals
		- TIM3 for PWM 1
		- TIM5 for PWM2
	- ADC1 (ANA0) enable for Cortex M4 for [[String Potentiometer]] Reading (in progress)
		- Notes:
			- ADC1_INP0 and ADC2_INP0 on pin ANA0 (Arduino connector A2)
			- ADC1_INP1 and ADC2_INP0 on pin ANA1 (Arduino connector A3)
			- ADC1_INP6 on pin PF12 (Arduino connector A5)
			- ADC1_INP13 on pin PC3 (Arduino connector A4)
			- ADC2_INP2 on pin PF13 (Arduino connector A1)
			- ADC2_INP6 on PF14 (Arduino connector A0)
	- IPCC for Cortex M4 enabled
		- Enable NVIC Settings for 
			- IPCC RX1 occupied interrupt
			- IPCC TX1 free interrupt
	- OpenAMP for Cortex M4 enabled for IPC between Cortex A7 and Cortex M4 (In Progress)
		- Notes:
			- Use tty_echo example

### CubeIDE
