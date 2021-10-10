#!/bin/bash

# always using the last Ethernet Controller as EtherCAT

INSTANCE=$(lspci -mm | grep -i 'Ethernet controller' | sed -n '$p' | cut -d ' ' -f 1 | awk '$1=$1')

KERNEL_MODULE=$(lspci -k | grep -A 3 -i 'Ethernet controller' | grep -i 'Kernel modules' | sed -n '$p' | cut -d ':' -f 2 | awk '$1=$1')

echo "============EtherCAT Master for Robot=============="
echo "===@Author: Yang Luo                            ==="
echo "===@Create: 2021.07.17 00:05                    ==="
echo "===@Ver: 0.1                                    ==="
echo "==================================================="
echo "The EtherCAT will use $INSTANCE as physical device."
echo "The kernel module of $INSTANCE is $KERNEL_MODULE."
echo "Please make sure that your network driver is supported by Ec-Master."

# uninstall the driver of the ethernet
echo "0000:$INSTANCE" | sudo tee -a /sys/bus/pci/drivers/$KERNEL_MODULE/unbind

if test -z "$(lsmod | grep atemsys)"; then
  sudo insmod atemsys.ko
fi

# Start EcMaster
# nice -20 ./rocos_ethercat -f $ENI_FILE -i8254x $INSTANCE 1 -v 3 -perf -t 0
