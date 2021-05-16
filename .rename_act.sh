#!/bin/bash

this=`basename "$0"`

for file in `find . -path ./.git -prune -o -path ./doc -prune -o ! -name "$this" -type f -print`
do
	awk '{gsub(/rt2_assignment1/, "rt2_assignment1_action"); print $0}' $file > .tmp && mv .tmp $file; \
	awk '{gsub(/position_service/, "position_service_a"); print $0}' $file > .tmp && mv .tmp $file; \
	awk '{gsub(/position_service_a.cpp/, "position_service.cpp"); print $0}' $file > .tmp && mv .tmp $file; \
	awk '{gsub(/state_machine/, "state_machine_a"); print $0}' $file > .tmp && mv .tmp $file; \
	awk '{gsub(/state_machine_a.cpp/, "state_machine.cpp"); print $0}' $file > .tmp && mv .tmp $file; \
done
