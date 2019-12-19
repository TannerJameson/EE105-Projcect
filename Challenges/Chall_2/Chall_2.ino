//In setup: a) write values for desired_angle kp, b) initialize input values from IMU for angle readings,
//and c) initialize output pins for right and left turns.

void Chall_2(angle, desired_angle, kp) {

  //Determine error
  change = angle-desired_angle;

  //Write gas to motor pin.
  gas = change*kp;

 //Make sure it's between -255 and 255.
 if gas > 255
    gas = 255;
 else if gas < -255
    gas = -255;
 end

  //Turn left or right.
  if gas > 0
      go(right, abs(change));
  else
      go(left, abs(change));
  end

  
}
