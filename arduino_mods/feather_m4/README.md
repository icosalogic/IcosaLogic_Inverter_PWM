# Arduino Mods for Adafruit Feather M4
The files are modified versions of files in the folder (on Linux):

```
~/.arduino15/packages/adafruit/hardware/samd/1.7.16/variants/feather_m4
```

Note that in the future, the version name on that path may change.

These changes add virtual pins 40 and 41, which are clones of pins 8 (A2) and 9 (A3),
with the difference that they use ADC1 instead of ADC0.
Without this change, none of the analog pins on the Feather M4 use ADC1.

To install these files, first move the existing variant.* files:

```
mv variant.h variant.h.orig
mv variant.cpp variant.cpp.orig
```

After copying the variant.* files in this folder to the directory above,
it would be prudent to exit the Arduino IDE and restart it.

