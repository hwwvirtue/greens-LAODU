int angletransmit(int angle, int target)
{
  /*---------------化为0到360---------------*/
  int A;
  if (-90 <= angle && angle <= 180)
    angle = angle + 90;
  else
    angle = angle + 450;
  /*----------转化为劣弧------------*/
  if (target - angle >= 0 && target - angle <= 180)
    A = target;
  if (target - angle > 180)
    A = 360 - (target - angle);
  else
    A = angle - target;
  return A;
}
