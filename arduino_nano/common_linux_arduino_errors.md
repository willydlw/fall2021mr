# Common Errors and Solutions 


## 1. Ubuntu Permssion Error - can't open device 

The first time you try uploading a program, you may see the following error.

```
avrdude: ser_open(): can't open device "/dev/ttyACM0": Permission denied
```

In a terminal, with the board plugged in, type 

```
ls -l /dev/ttyACM*
```

Output

```
crw-rw---- 1 root dialout 166, 0 Aug 28 13:04 /dev/ttyACM0
```



This says root is the owner of the file /dev/ttyACM0 and the dialout group has access privileges. The rw stand for read write privileges. 

This may be solved by
- adding user read write privileges (temporary fix)
- adding the user to dialout group (permanent fix)
- both of the above


#### Adding User Read Write Privileges

This is a temporary fix that will work as long as the board remains plugged in and Ubuntu is not shutdown and restarted. In the terminal, type the commands to add read write privileges and then list the file information again to see they have been added

```
sudo chmod a+rw /dev/ttyACM0
ls -l /dev/ttyACM*
```

The output should now resemble that below.

Output
```
crw-rw-rw- 1 root dialout 166, 0 Aug 28 13:04 /dev/ttyACM0
```

Try uploading the Arduino file again. If you still get an error, try the next step of adding the user to the dialout group.

#### Add User to dialout group

First, let's see if you are a member of the dialout group. In the terminal, type the command `groups` to show to which groups you, the user, belong.

The output shown below indicates that the user diane is not a member of the dialout group. 

```
diane adm cdrom sudo dip plugdev lpadmin sambashare docker
```

To add a user to the dialout group, type the following command, but substitute your user name in place of the words username.

```
sudo usermod -a -G dialout $USER
```

Typing groups may not show you have been added to the dialout group. There have been times where I have had to reboot Linux to have the group addition update. But first, try uploading the Arduino program. 
