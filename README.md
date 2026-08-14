# Chromatic FPGA

*Note: This is not official Chromatic code; this is a hobby expansion of it.*

This repository houses the FPGA design files for the ModRetro Chromatic hobby expansion.

For more information about the official ModRetro Chromatic hardware, please visit [ModRetro.com](https://modretro.com/).

## Release Notes: Core Update - Final

**Highlights:** Super Game Boy with basic SFX support and Overclocking are here! 

### ✨ New Features
*   **Easy Setup:** Added the new [Chromatic Installer](https://github.com/tmagri/chromatic_installer) for a streamlined, user-friendly installation process.
*   **Super Game Boy (SGB) Support:** 
    *   Added SGB Palettes.
    *   Added Basic SGB SFX, including Pauline's cry in *Donkey Kong '94* and sound effects (all 72 sfx) like *Kirby's Dream Land 2*. No SGB Music.
    *   *Note: Your MCU must be updated to support SGB SFX. Use the installer to flash both, for ease of use*
*   **Overclocking:** Introduced **2x** and **4x** speed modes. *Note: Overclocking is defined here as reducing game slowdown. It does not do fast-forward or increase the overall speed of the game, as we are limited by the read speed of the cartridge.* 

### 🛑 Project Status: Feature Freeze (Final)
The FPGA chip has reached 97% utilisation and is now routing and logic bound. This hardware limitation means adding new features is no longer feasible, and this sgb_sound branch/project is final. It has all the features intended for this expansion.

---

## ⚠️ CAUTION: Overclock Mode & EverDrive Risks
*Please read carefully before using the new overclocking features.*

*   **General Instability:** Consider Overclock mode to be inherently unstable at all times, especially when running at 4x speed.
*   **Menu Boot Failures:** **Do NOT use 4x mode while in the EverDrive menu**, or your games will fail to boot. It is strongly advised to only enable 4x mode *after* you are actively running a game.
*   **SD Card Corruption Risk:** Changing the overclock speed during a card read or write may cause severe system instability. Timing issues with EverDrive carts during these states can result in SD card corruption. 
*   **Manual Precautions:** Although the system logic automatically checks for cartridge activity, you should manually ensure no speed changes are made during read/write operations as an extra precaution. 

*(Note: No liability is accepted for potential data loss or SD card corruption).*

---

## Setup

### Repository

This project builds upon the open source work provided by the Game Boy `MiSTer` project. When checking out this repository, make sure to run the following command as this repository submodules the Game Boy `MiSTer` project.

```bash
git submodule update --init --recursive
```

### Gowin Development Environment

**The Gowin FPGA Designer v1.9.9.03 must be used.** Using Gowin IDE v1.9.10.X or newer is currently not supported by this build.

You will also need to apply for a local license with Gowin through their website:
https://www.gowinsemi.com/en/support/license

The license expires after one year and will require reactivation.

You will receive an email within a few minutes with a `.lic` file attached. Run the Gowin IDE and install the license when it prompts you. You'll need to close and re-open the GOWIN IDE if everything was successful.

## Building
Once in the IDE, load `evt1_x2.gprj` project and click on the green recycle-like button icon to run synthesis and PnR. This will take about 5-10 minutes to complete.

## Flashing
Flashing can be performed using the official [Gowin Programmer](https://www.gowinsemi.com/en/) software or the [`openFPGALoader`](https://github.com/trabucayre/openFPGALoader) utility through the Chromatic's USB interface. The Gowin Programmer requires the installation of the GWU2X device driver.

Note:
1. The Chromatic must be powered on for either tool to detect the FPGA. This means the power switch is in the **ON** position.
2. If using `openFPGALoader`, the tool must be compiled with support for the Gowin GWU2X cable.

### Example Using `openFPGALoader`
**Detect the Chromatic FPGA While Powered On**
```bash
openFPGALoader --detect --cable gwu2x
```

You will see an output similar to:
```
empty
User requested: 6000000 real frequency is 6000000
index 0:
        idcode 0x1281b
        manufacturer Gowin
        family GW5A
        model  GW5A-25
        irlength 8
```

**Flashing the Chromatic**

```bash
openFPGALoader --write-flash --cable gwu2x --reset <file>
```

Here, `<file>` refers to the generated bitstream file. This file can be found at `esp32t/impl/pnr/evt1_x2.fs`.

## Custom Modifications

When modifying the RTL design, please also update the 14-bit FPGA version within [esp32t/src/rtl/BSP/system_monitor.sv] around line 384 (see `version`).

This will ensure you can always using the [ModRetro Update Tool](https://modretro.com/pages/downloads#mrupdater) to restore your Chromatic to the latest official release.

## Issues
Please submit all issues and bug reports through our [Contact Form](https://modretro.com/pages/contact).

## Attributions
- [GOWIN Semiconductor](https://www.gowinsemi.com/en/)
- [MiSTer](https://github.com/MiSTer-devel/Gameboy_MiSTer)

## Special Thanks
- [rayjt9] For their palette improvements to the BootROM.
