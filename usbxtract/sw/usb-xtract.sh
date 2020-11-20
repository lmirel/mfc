
while true; do sudo /opt/mfc/usb-xtract --tty /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 --device 0eb7:0e04; echo "waiting for wheel.."; sleep 5; done
