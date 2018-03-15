//***********************************/
//*		  Auto-docking system		*/
//*				  -					*/
//* © FAITO Aerospace Inc.  -  2059 */
//***********************************/

//	Author : HerrCrazi

//This document is open-source, you can redistribute and modify it as you want,
//but don't forget to mention the author.


parameter debug is false.	//Set this to true to see some vectors on the screen showing the ship's speed & position


function vproj
{
	parameter a.
	parameter b.

	return vdot(a, b) / b:mag.
}

function getSignedMag
{
	parameter vect.
	parameter baseVect.

	local vangle is vang(vect, baseVect).
	if ( vangle > 90 and vangle < 270 )
	{
		return vect:mag.
	} else {
		return -vect:mag.
	}
}



clearvecdraws().
clearscreen.

//PID loop controlling the translation of the vessel
//Tweaking the PID : 
//	- Increase the P factor to increase the strengh of the corection. The higher it is, the harder the correction will be (and consume more fuel)
//	- Increasing the I factor will increase the importance of past errors in the correction. It can help bringing the vessel in line with the target
//	- Increasing the D factor will make the PID "predict" the future error and adapt the correction to that. This factor will make it slow slightly 
//	  before reaching the setpoint, which improves precision.

set PID_y to pidloop(8,12,4,-1,1).
set PID_y:setpoint to 0.

set PID_z to pidloop(8,12,4,-1,1).
set PID_z:setpoint to 0.

set Kv to 1. 	//Increment this value to slow the translation speed of the vessel. The speed is proportionnal to the distance from the target.

rcs on.
sas off.



if defined target and target:targetable
{
	set trgt to target.
	print trgt:name.

	set controllingPart to ship:controlpart.

	//Lock steering to the same orientation than the target's port, but in the opposite direction (so the ship is facing the target port)
	lock steering to lookdirup(-trgt:portfacing:forevector, trgt:portfacing:upvector).
	wait until vang(ship:facing:forevector, -trgt:portfacing:forevector) < 1 and vang(ship:facing:upvector, trgt:portfacing:upvector) < 5.

	//Target-centered base, with the x axis pointing forward out of the target
	lock vtx to trgt:portfacing:forevector.
	lock vty to trgt:portfacing:upvector.		//y axis is straight up out of the target
	lock vtz to trgt:portfacing:rightvector.	//z axis is starboard out of the target

	//Ship and target position in SHIP-RAW coordinates
	lock spos to ship:controlpart:position.
	lock tpos to trgt:position.

	lock targetVel to trgt:ship:velocity:orbit - ship:velocity:orbit.	//Target-relative velocity

	//Declared here for the when statement below
	declare trgt_y is 1.
	declare trgt_z is 1.

	set stop to false.

	

	on AG1
	{
		toggle stop.
	}

	//At this point we're aligned with the target, it's time to burn forward
	when ( abs(trgt_z) < 0.1 and abs(trgt_y) < 0.1 ) then
	{
		set ship:control:fore to 1.
		wait until ( targetVel:mag >= 1 ).
		set ship:control:fore to 0.

		return false.
	}



	until stop or ( trgt:state <> "Ready" and trgt:state <> "PreAttached" )//Press AG1 to stop at any moment
	{
		set v_ShipToTarget to tpos - spos.	//A vector going from the ship to the target

		//Convert the position vector of the ship relative to the target from SHIP-RAW to target-relative coordinates
		set trgt_x to getSignedMag(vtx * vproj(v_ShipToTarget, vtx), vtx).
		set trgt_y to getSignedMag(vty * vproj(v_ShipToTarget, vty), vty).
		set trgt_z to getSignedMag(vtz * vproj(v_ShipToTarget, vtz), vtz).
		//Same for their relative velocity
		set targetVel_x to -getSignedMag(vtx * vproj(targetVel, vtx), vtx).
		set targetVel_y to -getSignedMag(vty * vproj(targetVel, vty), vty).
		set targetVel_z to -getSignedMag(vtz * vproj(targetVel, vtz), vtz).

		//Draw some fancy vectors
		if debug
		{
			SET vdt TO VECDRAW(trgt:position, (-trgt:portfacing:forevector) * 5, yellow, "Trgt", 1, true).
			SET vds TO VECDRAW(V(0,0,0), (ship:facing:forevector) * 5, yellow, "Ship", 1, true).

			set x to VECDRAW( V(0,0,0),-vtx * trgt_x , red, "x", 1, true, 0.1).
			set y to VECDRAW( -vtx * trgt_x, -vty * trgt_y , green, "y", 1, true, 0.1).
			set z to VECDRAW( -vtx * trgt_x - vty * trgt_y,  -vtz * trgt_z , blue, "z", 1, true, 0.1).

			set tx to VECDRAW( V(0,0,0), -vtx * targetVel_x * 10, red, "Tx", 1, true, 0.05).
			set ty to VECDRAW( V(0,0,0), -vty * targetVel_y * 10, green, "Ty", 1, true, 0.05).
			set tz to VECDRAW( V(0,0,0), -vtz * targetVel_z * 10, blue, "Tz", 1, true, 0.05).
		}


		//Apply PID corrections
		set Vy to min(max(trgt_y*2,1),10).
		set Vz to min(max(trgt_z*2,1),10).

		set PID_y:setpoint to ( trgt_y / Vy ) * Kv.
		set ship:control:top to -PID_y:update(time:seconds, targetVel_y).

		set PID_z:setpoint to ( trgt_z / Vz ) * Kv.
		set ship:control:starboard to PID_z:update(time:seconds, targetVel_z).


		//Print stuff
		print "Corr. X  : " + round(0,3) + "      " at (0, terminal:height - 9).
		print "Corr. Y  : " + round(PID_y:output,3) + " (" + Vy + ")      " at (0, terminal:height - 8).
		print "Corr. Z  : " + round(PID_z:output, 3) + " (" + Vz + ")      " at (0, terminal:height - 7).
 
		print "Vel. X   : " + round(targetVel_x, 3) + "m/s      " at (terminal:width / 2, terminal:height - 9).
		print "Vel. Y   : " + round(targetVel_y, 3) + "m/s      " at (terminal:width / 2, terminal:height - 8).
		print "Vel. Z   : " + round(targetVel_z, 3) + "m/s      " at (terminal:width / 2, terminal:height - 7).

		print "Dist.    : " + round(trgt_x, 3) + "m      " at (0, terminal:height - 5).
		print "Right    : " + round(trgt_z, 3) + "m      " at (0, terminal:height - 4).
		print "Up       : " + round(trgt_y, 3) + "m      " at (0, terminal:height - 3).

		print "Rel. ang : "+vang( (-trgt:portfacing:forevector), ship:facing:forevector ) at (0, terminal:height - 1).
		wait 0.1.
	}



	unlock all.
	set ship:control:neutralize to true.

	sas on.
	clearvecdraws().
}
