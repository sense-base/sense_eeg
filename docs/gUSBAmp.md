# Installing dependencies for g.USBAmp

The following steps are based on [rosneuro_acquisition_eegdev](https://github.com/rosneuro/rosneuro_acquisition_eegdev).

## libeegdev-dev
STEP 1. Check OS distro and download server (Germany Julich Research Center; Germany G-Node)
```
# lsb_release -a #Description:	Ubuntu 22.04.5 LTS
# all packages are [DSFG](http://www.debian.org/social_contract#guidelines)-compliant, with permission to use, modify, re-distribute under any condition
# ALTERNATIVELY: individual packages may have restrictive licenses and you are required to check license-compliance manually
wget -O- http://neuro.debian.net/lists/jammy.de-m.full | sudo tee /etc/apt/sources.list.d/neurodebian.sources.list

cd ~/Downloads/
wget -O- http://neuro.debian.net/lists/jammy.de-m.libre | sudo tee /etc/apt/sources.list.d/neurodebian.sources.list
sudo apt-key adv --recv-keys --keyserver hkps://keyserver.ubuntu.com 0xA5D32F012649A5A9
sudo apt-get update
```

STEP 2: To install the libeegdev-dev
```
sudo apt-get install libeegdev-dev
```

## eegdev library
* Clone repository
```
sudo apt-get install libtool gnulib flex bison
cd $HOME//sense-base
git@github.com:mmlabs-mindmaze/eegdev.git
```

* Create `Local` path and add `.profile`
```
cd $HOME && mkdir -p Local
#vim .profile
if [ -d "$HOME/Local" ] ; then
    export PATH="$HOME/Local/bin:$PATH"
    export LD_LIBRARY_PATH="$HOME/Local/lib/:$LD_LIBRARY_PATH"
    export LIBRARY_PATH="$HOME/Local/lib/:$LIBRARY_PATH"
    export CPATH="$HOME/Local/include/:$CPATH"
    export PKG_CONFIG_PATH="$HOME/Local/lib/pkgconfig/:$PKG_CONFIG_PATH"
    export LIB=$LD_LIBRARY_PATH:$LIB
    export INCLUDE=$HOME/Local/include/:$INCLUDE
fi
```

* `./autogen.sh`

```
$ ./autogen.sh
libtoolize: putting auxiliary files in AC_CONFIG_AUX_DIR, 'build-aux'.
libtoolize: copying file 'build-aux/ltmain.sh'
libtoolize: putting macros in AC_CONFIG_MACRO_DIRS, 'm4'.
libtoolize: copying file 'm4/libtool.m4'
libtoolize: copying file 'm4/ltoptions.m4'
libtoolize: copying file 'm4/ltsugar.m4'
libtoolize: copying file 'm4/ltversion.m4'
libtoolize: copying file 'm4/lt~obsolete.m4'
aclocal: installing 'm4/ltdl.m4' from '/usr/share/aclocal/ltdl.m4'
configure.ac:31: warning: The macro `AC_PROG_CC_C99' is obsolete.
configure.ac:31: You should run autoupdate.

...


./lib/autoconf/general.m4:204: AC_HELP_STRING is expanded from...
configure.ac:170: the top level
configure.ac:31: installing 'build-aux/compile'
configure.ac:23: installing 'build-aux/missing'
doc/Makefile.am:57: warning: wildcard $(SRCTREE: non-POSIX variable name
doc/Makefile.am:57: (probably a GNU make extension)
doc/Makefile.am:64: warning: shell grep -c WARNING: sphinx-build.log: non-POSIX variable name
doc/Makefile.am:64: (probably a GNU make extension)
doc/Makefile.am:77: warning: wildcard $(SRCTREE: non-POSIX variable name
doc/Makefile.am:77: (probably a GNU make extension)
doc/Makefile.am:94: warning: foreach manpage, $(notdir $(wildcard $(builddir: non-POSIX variable name
doc/Makefile.am:94: (probably a GNU make extension)
doc/Makefile.am:98: warning: foreach manpage, $(notdir $(wildcard $(builddir: non-POSIX variable name
doc/Makefile.am:98: (probably a GNU make extension)
doc/examples/Makefile.am: installing 'build-aux/depcomp'
```



* `./configure --help`
```
`configure' configures eegdev 1.0 to adapt to many kinds of systems.

Usage: ./configure [OPTION]... [VAR=VALUE]...

To assign environment variables (e.g., CC, CFLAGS...), specify them as
VAR=VALUE.  See below for descriptions of some of the useful variables.

...

  --with-gtec             Support for gTec backend [default=check]

...
```


*  `./configure --prefix=$HOME/Local --with-gtec`
```

checking pkg-config is at least version 0.9.0... yes
checking for libusb-1.0... no
checking for library containing GT_OpenDevice... no
configure: error: in `/home/mxochicale/sense-base/eegdev_mmlabs-mindmaze':
configure: error: gUSBampAPI library required for gTec support
See `config.log' for more details


```


* Compile and install the library
```
make && make install
```

## build gtec driver
```
git clone git@github.com:mxochicale/openbci.git
cd openbci/obci/drivers/eeg/cpp_amplifiers/gtec
make clean
make gtec_amplifier
# Some steps are https://github.com/mxochicale/openbci/tree/master/obci/drivers/eeg/cpp_amplifiers/gtec
```