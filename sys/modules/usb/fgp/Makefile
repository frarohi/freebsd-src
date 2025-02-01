# $FreeBSD$

S=     ${SRCTOP}/sys

.PATH: $S/dev/usb/input

#CFLAGS= -DEVDEV_SUPPORT -DUSB_DEBUG
#CFLAGS= -DEVDEV_SUPPORT
KMOD=	fgp
SRCS=	opt_bus.h opt_evdev.h opt_usb.h device_if.h bus_if.h usb_if.h vnode_if.h usbdevs.h \
	fgp.c

.include <bsd.kmod.mk>
