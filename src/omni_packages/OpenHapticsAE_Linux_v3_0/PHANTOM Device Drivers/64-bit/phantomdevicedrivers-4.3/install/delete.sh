#!/bin/sh
/sbin/ldconfig
if [ $1 = 0 ] ; then
    rm -f /dev/phnepp
fi