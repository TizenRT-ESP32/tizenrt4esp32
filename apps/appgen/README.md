# TizenRT Application Generator

### Requirements
- Latest bash
- Generally, Ubuntu linux has the latest bash

### How to use?

In any directory, run the appgen shell script and just enter the application name that you would make.

Your application would be generated at the apps/examples/

```sh
TizenRT/os$ tools/appgen.sh
TizenRT Application Generator
======================= v 1.0
Enter application name: test application
[Summary]
-------------------------------
* Application Name: test application
* Configuration Key: CONFIG_APP_TEST_APPLICATION
* Entry Function: test_application_main
* Location: /home/gcjjyy/workspace/github.com/gcjjyy/TizenRT/apps/examples/test_application
* This year: 2018
-------------------------------
Continue? (y/N): y
Generating...

* How to setup your application
Run) TizenRT/os/tools$ ./configure.sh <BOARD>/<CONFIG>
Run) TizenRT/os$ make menuconfig
1. Turn on your application in Application Configuration/Examples menu
2. Set the entry point to your application in Application Configuration menu
------------------------------
Done!!

```

### After generated
```sh
TizenRT/os$ make menuconfig
```

- Enter "Application Configuration/Examples"
- Turn on your application
- Set the entry point to your application
