#!/bin/bash

configureme() {
    ip link set can0 type can bitrate 500000
    ip link set can0 up
}

startme() {
    cansend can0 100#01ffffff
}

stopme() {
    cansend can0 100#02ffffff
}

setFL() {
    cansend can0 100#030200ffffffffff
}

setFR() {
    cansend can0 100#030300ffffffffff
}

setRL() {
    cansend can0 100#030401ffffffffff
}

setRR() {
    cansend can0 100#030501ffffffffff
}

case "$1" in
    configure)    configureme ;;
    start)        startme ;;
    stop)         stopme ;;
    restart)      stopme; startme ;;
    setFL)        setFL ;;
    setFR)        setFR ;;
    setRL)        setRL ;;
    setRR)        setRR ;;
    *) echo "usage: $0 configure|start|stop|restart|setFL|setFR|setRL|setRR|" >&2
       exit 1
       ;;
    esac
