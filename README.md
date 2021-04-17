# Stretch Robot Robot Raconteur Driver
Robot Raconteur is an object oriented robot communication library. Robot Raconteur stretch robot driver has most interface in common with stretch [python2 API](https://github.com/hello-robot/stretch_body).

## Prerequisite:
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/wiki/Download) (Follow instruction to download, depending on different OS)



## Instructions:

### Running RR driver
In order to start Robot Raconteur driver, 
* Install Robot Raconteur for python2 on Ubuntu 18.04 system in Stretch NUC
* Clone this repo by `https://github.com/hehonglu123/Stretch_RR.git`
* Start Robot Raconteur service by `python stretch_service.py`

Keep in mind the service is running on port `23232`

### Running RR client

Robot Raconteur client can run on most operating system and software, in this repo we have a python example for position commanded jogging and status reading.
