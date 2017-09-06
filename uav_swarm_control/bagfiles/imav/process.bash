#!/bin/bash          

for f in *.bag
do
	# Remove the sufix from the name of the file
	a="$f"
	sufix=${a#*_}
	sufix=${sufix%.*}

	# Use it to create the processed files
	rostopic echo -b $f -p /enable_control > processed/$sufix\_ec.txt
	rostopic echo -b $f -p /formation_points > processed/$sufix\_fp.txt
	rostopic echo -b $f -p /migration_point > processed/$sufix\_mp.txt
	rostopic echo -b $f -p /quadrotor_1/pose > processed/$sufix\_q1.txt
	rostopic echo -b $f -p /quadrotor_2/pose > processed/$sufix\_q2.txt
	rostopic echo -b $f -p /quadrotor_3/pose > processed/$sufix\_q3.txt
done

for f in processed/*.txt
do
	tail -n +2 "$f" > "$f.tmp" && mv "$f.tmp" "$f"
done
