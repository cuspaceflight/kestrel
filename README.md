Thrust vectoring repository

Setup
-----

* Install [scons](http://www.scons.org/download.php)
* Install [arduino-scons-alt](https://github.com/tomjnixon/arduino-scons-alt)

Building
--------

* cd into the relevant project folder
* `scons` will build
* `scons upload` will build and upload
* If you get an error about com ports, adjust the port in `SConstruct`
