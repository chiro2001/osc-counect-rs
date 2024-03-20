#!/usr/bin/env bash

type=debug
arch=thumbv7m-none-eabi
root_path=$(realpath "$(dirname "$0")"/..)
#gdb="$HOME/.local/share/JetBrains/Toolbox/apps/clion/bin/gdb/linux/x64/bin/gdb"
elf="$root_path/target/$arch/$type/osc-counect-rs"

nohup openocd -f "$root_path/openocd.cfg" -c "tcl_port disabled" -c "telnet_port disabled" -c "gdb_port 3333" -c "program \"$elf\"" -c reset -c halt &
openocd_pid=$!
echo "OpenOCD PID: $openocd_pid"
sleep 1