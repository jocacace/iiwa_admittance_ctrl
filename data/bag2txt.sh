#!/bin/bash

#Store all bagfile in text files
type=$1
rm -r $1
mkdir $1
#-------EMG /MYO/myo_emg
myo_emg0=$type"/myo_emg0_tmp"
myo_emg1=$type"/myo_emg1_tmp"
myo_emg2=$type"/myo_emg2_tmp"
myo_emg3=$type"/myo_emg3_tmp"
myo_emg4=$type"/myo_emg4_tmp"
myo_emg5=$type"/myo_emg5_tmp"
myo_emg6=$type"/myo_emg6_tmp"
myo_emg7=$type"/myo_emg7_tmp"
myo_emg7=$type"/myo_emg7_tmp"
myo_norm=$type"/myo_norm_tmp"
myo_emg=$type"/myo_emg.txt"

touch $myo_emg0
touch $myo_emg1
touch $myo_emg2
touch $myo_emg3
touch $myo_emg4
touch $myo_emg5
touch $myo_emg6
touch $myo_emg7
touch $myo_norm
rm $myo_emg
rostopic echo -p /MYO/myo_emg/data[0] >> $myo_emg0 & 
rostopic echo /MYO/myo_emg/data[1] >> $myo_emg1 & 
rostopic echo /MYO/myo_emg/data[2] >> $myo_emg2 & 
rostopic echo /MYO/myo_emg/data[3] >> $myo_emg3 & 
rostopic echo /MYO/myo_emg/data[4] >> $myo_emg4 & 
rostopic echo /MYO/myo_emg/data[5] >> $myo_emg5 & 
rostopic echo /MYO/myo_emg/data[6] >> $myo_emg6 & 
rostopic echo /MYO/myo_emg/data[7] >> $myo_emg7 & 
rostopic echo /MYO/myo_emg_norm >> $myo_norm & 
#----

#-------Attraction point /iiwa/attraction_point
ap_x=$type"/ap_x_tmp"
ap_y=$type"/ap_y_tmp"
ap_z=$type"/ap_z_tmp"
ap=$type"/ap.txt"
touch $ap_x
touch $ap_y
touch $ap_z
rm $ap

rostopic echo -p  /iiwa/attraction_point/x >> $ap_x & 
rostopic echo /iiwa/attraction_point/y >> $ap_y & 
rostopic echo /iiwa/attraction_point/z >> $ap_z & 
#------

#-------Distance from traj
dtraj_d=$type"/traj_dist_tmp"
touch $dtraj_d
rm $dtraj_d
dtraj=$type"/traj_dist"

rostopic echo -p /iiwa/dist_from_traj >> $dtraj_d &
#-------	

#-------Command position  /iiwa/command/CartesianPose
cmd_p_x=$type"/cmd_x_tmp"
cmd_p_y=$type"/cmd_y_tmp"
cmd_p_z=$type"/cmd_z_tmp"
cmd_p=$type"/cmd_p.txt"
touch $cmd_p_x
touch $cmd_p_y
touch $cmd_p_z
rm $cmd_p
rostopic echo  -p /iiwa/command/CartesianPose/pose/position/x >> $cmd_p_x & 
rostopic echo /iiwa/command/CartesianPose/pose/position/y >> $cmd_p_y & 
rostopic echo /iiwa/command/CartesianPose/pose/position/z >> $cmd_p_z & 
#------

#-------Position /iiwa/state/CartesianPose
eef_x=$type"/eef_x_tmp"
eef_y=$type"/eef_y_tmp"
eef_z=$type"/eef_z_tmp"
eef=$type"/eef.txt"
touch $eef_x
touch $eef_y
touch $eef_z
rm $eef

rostopic echo  -p /iiwa/state/CartesianPose/pose/position/x >> $eef_x & 
rostopic echo /iiwa/state/CartesianPose/pose/position/y >> $eef_y & 
rostopic echo /iiwa/state/CartesianPose/pose/position/z >> $eef_z & 
#------

#-------Human direction /iiwa/eef_h_direction
hdir_x=$type"/hdir_x_tmp"
hdir_y=$type"/hdir_y_tmp"
hdir_z=$type"/hdir_z_tmp"
hdir=$type"/hdir.txt"
touch $hdir_x
touch $hdir_y
touch $hdir_z
rm $hdir

rostopic echo -p  /iiwa/eef_h_direction/x >> $hdir_x & 
rostopic echo /iiwa/eef_h_direction/y >> $hdir_y & 
rostopic echo /iiwa/eef_h_direction/z >> $hdir_z & 
#------

#-------Planned direction /iiwa/eef_h_direction
pdir_x=$type"/pdir_x_tmp"
pdir_y=$type"/pdir_y_tmp"
pdir_z=$type"/pdir_z_tmp"
pdir=$type"/pdir.txt"
touch $pdir_x
touch $pdir_y
touch $pdir_z
rm $pdir
rostopic echo -p  /iiwa/eef_p_direction/x >> $pdir_x & 
rostopic echo /iiwa/eef_p_direction/y >> $pdir_y & 
rostopic echo /iiwa/eef_p_direction/z >> $pdir_z & 
#------

#-------next wp /iiwa/next_wp
nwp_x=$type"/nwp_x_tmp"
nwp_y=$type"/nwp_y_tmp"
nwp_z=$type"/nwp_z_tmp"
nwp=$type"/nwp.txt"
touch $nwp_x
touch $nwp_y
touch $nwp_z
rm $nwp

rostopic echo -p  /iiwa/next_wp/position/x >> $nwp_x & 
rostopic echo /iiwa/next_wp/position/y >> $nwp_y & 
rostopic echo /iiwa/next_wp/position/z >> $nwp_z & 
#------

#-------forces /netft_data/filt
f_x=$type"/f_x_tmp"
f_y=$type"/f_y_tmp"
f_z=$type"/f_z_tmp"
f=$type"/f.txt"
touch $f_x
touch $f_y
touch $f_z
rm $f

rostopic echo -p  /netft_data/filt/wrench/force/x >> $f_x & 
rostopic echo /netft_data/filt/wrench/force/y >> $f_y & 
rostopic echo /netft_data/filt/wrench/force/z >> $f_z & 
#------

#-------angle /iiwa/shared/angle
angle_d=$type"/angle_tmp"
angle=$type"/angle.txt"
touch $angle_d
rm $angle
touch $angle
rostopic echo -p  /iiwa/shared/angle/data >> $angle_d & 
#----

#-------hdir
hdir_x_d=$type"/hdir_x_tmp"
hdir_y_d=$type"/hdir_y_tmp"
hdir_z_d=$type"/hdir_z_tmp"
hdir=$type"/hdir.txt"
touch $hdir_d
rm $hdir
touch $hdir
rostopic echo -p /iiwa/eef_h_direction/x >> $hdir_x_d &
rostopic echo /iiwa/eef_h_direction/y >> $hdir_y_d &
rostopic echo /iiwa/eef_h_direction/z >> $hdir_z_d &
#---------



read -p "Press any key..." -n1 -s

exec 3<$myo_emg0
exec 4<$myo_emg1
exec 5<$myo_emg2
exec 6<$myo_emg3
exec 7<$myo_emg4
exec 8<$myo_emg5
exec 9<$myo_emg6
exec 10<$myo_emg7
while read l1 <&3 && read l2 <&4 && read l3 <&5  && read l4 <&6 && read l5 <&7  && read l6 <&8 && read l7 <&9 && read l8 <&10  
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		#echo "$l1 $l2 $l3 $l4 $l5 $l6 $l7 $l8" >> $myo_emg
		echo "$l1 $l2 $l3 $l4 $l5 $l6 $l7 $l8" >> $myo_emg
	fi	
done 
exec 3<&~
exec 4<&~
exec 5<&~
exec 6<&~
exec 7<&~
exec 8<&~
exec 9<&~
exec 10<&~

tail -n +2 $myo_emg > tmp
mv tmp $myo_emg

exec 3<$ap_x
exec 4<$ap_y
exec 5<$ap_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $ap
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~

tail -n +2 $ap > tmp
mv tmp $ap


exec 3<$cmd_p_x
exec 4<$cmd_p_y
exec 5<$cmd_p_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $cmd_p
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~

tail -n +2 $cmd_p > tmp
mv tmp $cmd_p

exec 3<$eef_x
exec 4<$eef_y
exec 5<$eef_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $eef
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~

tail -n +2 $eef > tmp
mv tmp $eef

exec 3<$hdir_x
exec 4<$hdir_y
exec 5<$hdir_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $hdir
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~

tail -n +2 $hdir > tmp
mv tmp $hdir


exec 3<$pdir_x
exec 4<$pdir_y
exec 5<$pdir_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $pdir
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~

tail -n +2 $pdir > tmp
mv tmp $pdir


exec 3<$nwp_x
exec 4<$nwp_y
exec 5<$nwp_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $nwp
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~


tail -n +2 $nwp > tmp
mv tmp $nwp


exec 3<$f_x
exec 4<$f_y
exec 5<$f_z
while read l1 <&3 && read l2 <&4 && read l3 <&5 
do
	l1="$l1"
	if [ "$l2" != "---" ]; then
		echo "$l1 $l2 $l3" >> $f
	fi
done 
exec 3<&~
exec 4<&~
exec 5<&~
#------

tail -n +2 $f > tmp
mv tmp $f

#-------angle /iiwa/shared/angle
exec 3<$angle_d
while read l1 <&3  
do
	l1="$l1"
	if [ "$l1" != "---" ]; then
		echo "$l1" >> $angle
	fi
done 
exec 3<&~

tail -n +2 $angle > tmp
mv tmp $angle
#------

tail -n +2 $dtraj_d > tmp
mv tmp $dtraj
#------


rm $myo_emg0
rm $myo_emg1
rm $myo_emg2
rm $myo_emg3
rm $myo_emg4
rm $myo_emg5
rm $myo_emg6
rm $myo_emg7
rm $myo_norm
rm $ap_x
rm $ap_y
rm $ap_z
rm $cmd_p_x
rm $cmd_p_y
rm $cmd_p_z
rm $eef_x
rm $eef_y
rm $eef_z
rm $hdir_x
rm $hdir_y
rm $hdir_z
rm $pdir_x
rm $pdir_y
rm $pdir_z
rm $nwp_x
rm $nwp_y
rm $nwp_z
rm $f_x
rm $f_y
rm $f_z
rm $angle_d
rm $dtraj_d


sed -i 's/,/ /g' $type/*

killall rostopic
