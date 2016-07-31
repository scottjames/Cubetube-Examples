# Cubetube-Examples

# Flashing photon firmware:

* Use dfu-util:
  * Fetch firmware from: https://github.com/spark/firmware/releases/tag/v0.5.2

```
dfu-util -d 2b04:d006 -a 0 -s 0x8020000 -D system-part1-0.5.2-photon.bin
dfu-util -d 2b04:d006 -a 0 -s 0x8060000:leave -D system-part2-0.5.2-photon.bin
 ```

* Flash tinker:
```
particle flash --usb tinker
```

# Keys

* Running:

```
particle keys doctor
```

can fix corrupted keys

# To compile ino files in the current working directory:

```
particle compile photon
particle flash --usb photon_firmware_XXXXXX.bin
```

# L3D source repos

These repos do not seem to work with the 16x16x16 cube

* https://github.com/enjrolas/Cubetube-Library
* https://github.com/enjrolas/L3D-Software
* https://github.com/enjrolas/L3D-Library
* https://github.com/enjrolas/L3D-Hardware

We have found some working 16x16x16 code on:

* http://old.cubetube.org/ 

# Our repos:

* https://github.com/srqsoftware/Cubetube-Examples
* https://github.com/srqsoftware/Cubetube-Library (fork)

