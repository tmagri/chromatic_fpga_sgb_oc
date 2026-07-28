# Chromatic FPGA

This repository houses the ModRetro Chromatic's FPGA design files.

For more information about the ModRetro Chromatic, please see visit [ModRetro.com](https://modretro.com/).

## Release Notes

- **Super Gameboy & Overclocking**: This revised core adds support for Super Gameboy and Overclocking features!
- **Feature Freeze**: The FPGA chip is currently at 97% utilization (routing and logic bound), meaning adding new features is no longer feasible. Moving forward, only bug fixes will be checked and merged.
- **Overclock Warning**: Consider Overclock mode to always be unstable, especially at 4x speed. The core automatically keeps the CPU at 1x during boot and while a detected flashcart OS (EverDrive) is running, but if you return to the flashcart's menu from a game (via its in-game menu), the overclock stays engaged — **browsing/loading from the EverDrive menu with 4x active can still make games fail to boot**. Power-cycle or re-insert the cart to re-arm the safe 1x boot window.
- **Safe-Boot Clock Override**: The core automatically forces the 1x clock during the boot and cartridge-loading sequence, so an overclock can be left enabled at power-on without hanging EverDrive carts on a white screen. For a normal cartridge the selected 2x/4x overclock engages the moment the game starts executing. For a detected flashcart (EverDrive X-series "GBXOS" and original-series "GBOS" headers), 1x is held until the OS actually boots a game from its menu, so OS initialisation, SD I/O, browsing and game loading all run at native speed.

## ⚠️ CAUTION: Overclock Mode

Changing the overclock speed during a card read or write may cause system instability. Please avoid making changes during these operations. Although the system logic automatically checks for cartridge activity, you should manually ensure no changes are made during these times as an extra precaution. Timing issues with EverDrive carts can result in SD card corruption. **No liability is accepted for potential data loss.**

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
