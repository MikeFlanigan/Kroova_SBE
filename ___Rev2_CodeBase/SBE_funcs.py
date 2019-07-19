'''
SBE Funcs
'''
import os
import datetime


def mount_usb():
    mounted_successfully = False
    unmounted_successfully = True
    failed_mount = False
    mount_timer = datetime.datetime.now()
    if not os.path.isfile('/media/usb/important_text.txt'):
        mount_check = os.system('sudo mount /dev/sda1 /media/usb')
        while True:
            if mount_check == 0:
                mounted_successfully = True
                print('usb mounted')
                break
            elif (datetime.datetime.now()-mount_timer).seconds > 15:
                print('failed to mount')
                failed_mount = True
                break
    elif os.path.isfile('/media/usb/important_text.txt'): print('usb already mounted')

def unmount_usb():
    mount_timer = datetime.datetime.now()
    if os.path.isfile('/media/usb/important_text.txt'):
        unmounted_successfully = False
        mount_check = os.system('sudo umount /media/usb')
        while True:
            if mount_check == 0:
                unmounted_successfully = True
                print('usb unmounted')
                break
            elif (datetime.datetime.now()-mount_timer).seconds > 15:
                failed_mount = True
                print('failed to unmount')
                break
    elif not os.path.isfile('/media/usb/important_text.txt'): print('usb already unmounted')
