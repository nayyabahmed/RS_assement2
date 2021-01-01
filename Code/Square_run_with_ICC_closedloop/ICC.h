void update_pose_ICC() {


 long delta_1 = count_e1 - old_count_e1;

 long delta_0 = count_e0 - old_count_e0;



 if (delta_1 == delta_0) {

  float avg_dist = ((delta_1 + delta_0) / 2) * MM_Per_Count;

  _xpos = _xpos + avg_dist * cos(theta * deg_rad);

  _ypos = _ypos + avg_dist * sin(theta * deg_rad);

 }



 else {

  float delta_theta = (delta_1 - delta_0) / Wheel_Seperation;

  float r_ICC = (Wheel_Seperation / 2) * ( (delta_1 + delta_0) / (delta_0 - delta_1) );

  _theta += delta_theta * rad_deg * MM_Per_Count;





  if (_theta > 360)_theta -= 360;

  else if (_theta < 0)_theta += 360;



  float x_ICC = _xpos - r_ICC * sin(_theta * deg_rad);

  float y_ICC = _ypos + r_ICC * cos(_theta * deg_rad);



  xpos = cos(delta_theta) * (_xpos - x_ICC) - sin(delta_theta) * (_ypos - y_ICC) + x_ICC;

  ypos = sin(delta_theta) * (_xpos - x_ICC) + cos(delta_theta) * (_xpos - y_ICC) + y_ICC;

 }
