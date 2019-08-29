void setup() 
{
  Serial.begin(9600);
  digitalWrite(52,INPUT);
  pinMode(52,LOW);
  digitalWrite(53,INPUT);
  pinMode(53,LOW);
}

void loop()
{
//  if(digitalRead(52) == HIGH)
//  {
//    Serial.println("No");
//  }
//  else if(digitalRead(52) == LOW)
//  {
//    Serial.println("Yes");
//  }

  if(digitalRead(52) == LOW && digitalRead(53) == HIGH)
  {
    Serial.println("shun");
  }
  else if(digitalRead(52) == HIGH && digitalRead(53) == LOW)
  {
     Serial.println("ni");
  }
  else
  {
    Serial.println("No");
  }
}
