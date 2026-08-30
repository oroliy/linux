#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only

set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_dir=$(dirname -- "$script_dir")
port_dir=$(dirname -- "$repo_dir")
# The initramfs harness lives outside the kernel tree (not part of the kernel
# sources); override with X6818_BOARD_DIR if it moves.
if [ -n "${X6818_BOARD_DIR:-}" ]; then
	board_dir=$X6818_BOARD_DIR
else
	common_git_dir=$(git -C "$repo_dir" rev-parse --path-format=absolute \
		--git-common-dir 2>/dev/null || true)
	if [ -n "$common_git_dir" ]; then
		port_dir=$(dirname -- "$(dirname -- "$common_git_dir")")
	fi
	board_dir=$port_dir/x6818-initramfs
fi
legacy_rootfs=${X6818_ROOTFS:-"$port_dir/rootfs"}
output=${1:-"$repo_dir/arch/arm/boot/x6818-initramfs.cpio.gz"}
stage=$(mktemp -d "${TMPDIR:-/tmp}/x6818-initramfs.XXXXXX")

cleanup()
{
	rm -rf "$stage"
}
trap cleanup EXIT HUP INT TERM

mkdir -p "$stage"/bin "$stage"/dev "$stage"/proc "$stage"/sys
mkdir -p "$stage"/tmp "$stage"/lib "$stage"/sbin
install -m 0755 "$board_dir/init" "$stage/init"
install -m 0755 "$legacy_rootfs/bin/busybox" "$stage/bin/busybox"
install -m 0755 "$legacy_rootfs/lib/ld-linux.so.3" "$stage/lib/ld-linux.so.3"
install -m 0755 "$legacy_rootfs/lib/libc.so.6" "$stage/lib/libc.so.6"
install -m 0755 "$legacy_rootfs/lib/libm.so.6" "$stage/lib/libm.so.6"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-nostdlib -nostartfiles -nodefaultlibs -static \
	-march=armv7-a -marm -mfloat-abi=soft -fno-pie -no-pie \
	-Wl,--build-id=none -Wl,-e,_start \
	-o "$stage/bin/wdt-hang" "$board_dir/wdt-hang.S"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
	-o "$stage/bin/mmc-probe" "$board_dir/mmc-probe.c"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
		-fno-stack-protector \
	-o "$stage/bin/net-probe" "$board_dir/net-probe.c"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
		-fno-stack-protector \
	-o "$stage/bin/usb-probe" "$board_dir/usb-probe.c"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
		-fno-stack-protector \
	-o "$stage/bin/fb-probe" "$board_dir/fb-probe.c"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
		-fno-stack-protector \
	-o "$stage/bin/smp-probe" "$board_dir/smp-probe.c"

${CROSS_COMPILE:-arm-linux-gnueabihf-}gcc \
	-static -Os -march=armv7-a -marm -fno-pie -no-pie \
		-fno-stack-protector \
	-o "$stage/bin/smp-stress" "$board_dir/smp-stress.c"

ln -s busybox "$stage/bin/sh"
ln -s ../bin/busybox "$stage/sbin/reboot"
ln -s ../bin/busybox "$stage/sbin/halt"
ln -s ../bin/busybox "$stage/sbin/poweroff"

mkdir -p "$(dirname -- "$output")"
(
	cd "$stage"
	# Keep watchdog validation independent of devtmpfs and the legacy BusyBox.
	fakeroot sh -c '
		mknod dev/watchdog c 10 130
		mknod dev/mmcblk0 b 179 0
		mknod dev/fb0 c 29 0
		find . -print | cpio --quiet -o -H newc --owner=0:0
	'
) | gzip -9 > "$output"

printf 'built %s (%s bytes)\n' "$output" "$(stat -c %s "$output")"
