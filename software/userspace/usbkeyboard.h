#ifndef _USBKEYBOARD_H
#define _USBKEYBOARD_H

#include <libusb-1.0/libusb.h>

#define USB_HID_KEYBOARD_PROTOCOL 1

/* Modifier bits */
#define USB_LCTRL  (1 << 0)
#define USB_LSHIFT (1 << 1)
#define USB_LALT   (1 << 2)
#define USB_LGUI   (1 << 3)
#define USB_RCTRL  (1 << 4)
#define USB_RSHIFT (1 << 5)
#define USB_RALT   (1 << 6)
#define USB_RGUI   (1 << 7)

struct usb_keyboard_packet {
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keycode[6];
};

/* HID scan codes used by the demo */
#define KEY_A      0x04
#define KEY_B      0x05
#define KEY_C      0x06
#define KEY_D      0x07
#define KEY_E      0x08
#define KEY_F      0x09
#define KEY_G      0x0a
#define KEY_H      0x0b
#define KEY_I      0x0c
#define KEY_J      0x0d
#define KEY_K      0x0e
#define KEY_L      0x0f
#define KEY_M      0x10
#define KEY_N      0x11
#define KEY_O      0x12
#define KEY_P      0x13
#define KEY_Q      0x14
#define KEY_R      0x15
#define KEY_S      0x16
#define KEY_T      0x17
#define KEY_U      0x18
#define KEY_V      0x19
#define KEY_W      0x1a
#define KEY_X      0x1b
#define KEY_Y      0x1c
#define KEY_Z      0x1d

#define KEY_1      0x1e
#define KEY_2      0x1f
#define KEY_3      0x20
#define KEY_4      0x21
#define KEY_5      0x22
#define KEY_6      0x23
#define KEY_7      0x24
#define KEY_8      0x25
#define KEY_9      0x26
#define KEY_0      0x27

#define KEY_ENTER      0x28
#define KEY_ESC        0x29
#define KEY_BACKSPACE  0x2a
#define KEY_TAB        0x2b
#define KEY_SPACE      0x2c
#define KEY_MINUS      0x2d
#define KEY_EQUAL      0x2e

#define KEY_UP    0x52
#define KEY_DOWN  0x51
#define KEY_LEFT  0x50
#define KEY_RIGHT 0x4f

/* Find and open a USB keyboard. Argument receives the endpoint address.
   Returns NULL if no keyboard found. */
extern struct libusb_device_handle *openkeyboard(uint8_t *endpoint_address);

#endif /* _USBKEYBOARD_H */
