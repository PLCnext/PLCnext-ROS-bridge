#!/bin/ash
if [ ! -e "/www/index.html" ]; then
    cp index.html /www
fi
echo "httpd started" && trap "exit 0;" TERM INT; httpd -v -p $PORT -h /www -f & wait