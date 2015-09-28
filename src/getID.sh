#!/bin/bash

tmpfile=$(mktemp)

stty -F $1 raw -echo
sleep 1

cat < $1 > $tmpfile &
catpid=$!

echo PP > $1
echo VV > $1
sleep 0.2
kill "$catpid"

grep SERI $tmpfile 2> /dev/null | sed -e "s/^SERI: *\([0-9a-zA-Z]*\);.*$/\1/"

