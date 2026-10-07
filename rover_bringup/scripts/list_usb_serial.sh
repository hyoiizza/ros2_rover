#!/bin/bash
# Print the identity of every USB-serial device currently attached, plus a
# ready-to-paste udev rule for each one.
#
# Use this to fill in rover_bringup/udev/99-rover-sensors.rules when the
# RPLIDAR C1 adapter and the LX-16A BusLinker turn out to use the same chip.

shopt -s nullglob
devices=(/dev/ttyUSB* /dev/ttyACM*)

if [ ${#devices[@]} -eq 0 ]; then
  echo "No /dev/ttyUSB* or /dev/ttyACM* devices found."
  exit 1
fi

for dev in "${devices[@]}"; do
  info=$(udevadm info -q property -n "$dev")
  vendor=$(echo "$info" | sed -n 's/^ID_VENDOR_ID=//p')
  product=$(echo "$info" | sed -n 's/^ID_MODEL_ID=//p')
  serial=$(echo "$info" | sed -n 's/^ID_SERIAL_SHORT=//p')
  model=$(echo "$info" | sed -n 's/^ID_MODEL=//p')
  links=$(echo "$info" | sed -n 's/^DEVLINKS=//p')
  kernels=$(udevadm info -q path -n "$dev" | grep -oE '[0-9]+-[0-9.]+' | tail -1)

  echo "=== $dev"
  echo "    chip        : $model ($vendor:$product)"
  echo "    serial      : ${serial:-<none reported>}"
  echo "    usb port    : ${kernels:-<unknown>}"
  echo "    symlinks    : ${links:-<none>}"
  if [ -n "$serial" ]; then
    echo "    udev rule   : KERNEL==\"${dev##*/tty}\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$product\", ATTRS{serial}==\"$serial\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"CHOOSE_A_NAME\""
  else
    echo "    udev rule   : KERNEL==\"tty*\", KERNELS==\"$kernels\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"CHOOSE_A_NAME\""
    echo "                  (no serial number reported, so this pins the physical USB socket)"
  fi
  echo
done
