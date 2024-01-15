.. _usb_cdc-acm:

USB CDC ACM Sample Application
####################################

Overview
********

| This sample app demonstrates use of a USB Communication Device Class (CDC)
Abstract Control Model (ACM) driver provided by the Zephyr project.
Received data from the serial port is echoed back to the same port
provided by this driver.

| The device is recognized as a serial device in windows. You can open the serial port in the serial communications software like sscom and test the firmware with it.

| When you send the string to the nRF52833, the nRF52833 will response the received string to the PC.
