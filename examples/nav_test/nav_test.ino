#include "Autopilot.h"




navigator myNavigator;




void setup()
{
  myNavigator.navFrame.ni.alt     = 10;
  myNavigator.navFrame.ni.lat     = 48.858273;
  myNavigator.navFrame.ni.lon     = 2.294524;
  myNavigator.navFrame.ni.heading = 0;
  myNavigator.navFrame.ni.speed   = 10;
  myNavigator.navFrame.ni.maxRoll = 30;
  
  myNavigator.navFrame.nf.alt     = 100;
  myNavigator.navFrame.nf.lat     = 48.856014;
  myNavigator.navFrame.nf.lon     = 2.297860;
  myNavigator.navFrame.nf.heading = 0;
  myNavigator.navFrame.nf.speed   = 10;
  myNavigator.navFrame.nf.maxRoll = 30;

  myNavigator.processFrame();
  
  Serial.print("path: ");
  if (myNavigator.navFrame.path == RSRU)
    Serial.println("RSRU");
  else if (myNavigator.navFrame.path == RSLU)
    Serial.println("RSLU");
  else if (myNavigator.navFrame.path == LSRU)
    Serial.println("LSRU");
  else if (myNavigator.navFrame.path == LSLU)
    Serial.println("LSLU");
  else if (myNavigator.navFrame.path == RSRD)
    Serial.println("RSRD");
  else if (myNavigator.navFrame.path == RSLD)
    Serial.println("RSLD");
  else if (myNavigator.navFrame.path == LSRD)
    Serial.println("LSRD");
  else if (myNavigator.navFrame.path == LSLD)
    Serial.println("LSLD");
  Serial.println();
  
  Serial.print("ni Altitude:\t");   Serial.println(myNavigator.navFrame.ni.alt,        20);
  Serial.print("ni maxRoll:\t");    Serial.println(myNavigator.navFrame.ni.maxRoll,    20);
  Serial.print("ni minTurnRad:\t"); Serial.println(myNavigator.navFrame.ni.minTurnRad, 20);
  Serial.print("ni hitRadius:\t");  Serial.println(myNavigator.navFrame.ni.hitRadius,  20);
  Serial.print("ni speed:\t");      Serial.println(myNavigator.navFrame.ni.speed,      20);
  Serial.print("ni heading:\t");    Serial.println(myNavigator.navFrame.ni.heading,    20);
  Serial.print("ni lat:\t\t");      Serial.println(myNavigator.navFrame.ni.lat,        20);
  Serial.print("ni lon:\t\t");      Serial.println(myNavigator.navFrame.ni.lon,        20);
  Serial.print("ni rc_lat:\t");     Serial.println(myNavigator.navFrame.ni.rc_lat,     20);
  Serial.print("ni rc_lon:\t");     Serial.println(myNavigator.navFrame.ni.rc_lon,     20);
  Serial.print("ni lc_lat:\t");     Serial.println(myNavigator.navFrame.ni.lc_lat,     20);
  Serial.print("ni lc_lon:\t");     Serial.println(myNavigator.navFrame.ni.lc_lon,     20);
  Serial.print("ni c_lat:\t");      Serial.println(myNavigator.navFrame.ni.c_lat,      20);
  Serial.print("ni c_lon:\t");      Serial.println(myNavigator.navFrame.ni.c_lon,      20);
  Serial.print("ni e_lat:\t");      Serial.println(myNavigator.navFrame.ni.e_lat,      20);
  Serial.print("ni e_lon:\t");      Serial.println(myNavigator.navFrame.ni.e_lon,      20);
  Serial.println();
  
  Serial.print("nf Altitude:\t");   Serial.println(myNavigator.navFrame.nf.alt,        20);
  Serial.print("nf maxRoll:\t");    Serial.println(myNavigator.navFrame.nf.maxRoll,    20);
  Serial.print("nf minTurnRad:\t"); Serial.println(myNavigator.navFrame.nf.minTurnRad, 20);
  Serial.print("nf hitRadius:\t");  Serial.println(myNavigator.navFrame.nf.hitRadius,  20);
  Serial.print("nf speed:\t");      Serial.println(myNavigator.navFrame.nf.speed,      20);
  Serial.print("nf heading:\t");    Serial.println(myNavigator.navFrame.nf.heading,    20);
  Serial.print("nf lat:\t\t");      Serial.println(myNavigator.navFrame.nf.lat,        20);
  Serial.print("nf lon:\t\t");      Serial.println(myNavigator.navFrame.nf.lon,        20);
  Serial.print("nf rc_lat:\t");     Serial.println(myNavigator.navFrame.nf.rc_lat,     20);
  Serial.print("nf rc_lon:\t");     Serial.println(myNavigator.navFrame.nf.rc_lon,     20);
  Serial.print("nf lc_lat:\t");     Serial.println(myNavigator.navFrame.nf.lc_lat,     20);
  Serial.print("nf lc_lon:\t");     Serial.println(myNavigator.navFrame.nf.lc_lon,     20);
  Serial.print("nf c_lat:\t");      Serial.println(myNavigator.navFrame.nf.c_lat,      20);
  Serial.print("nf c_lon:\t");      Serial.println(myNavigator.navFrame.nf.c_lon,      20);
  Serial.print("nf e_lat:\t");      Serial.println(myNavigator.navFrame.nf.e_lat,      20);
  Serial.print("nf e_lon:\t");      Serial.println(myNavigator.navFrame.nf.e_lon,      20);
  Serial.println();
}




void loop()
{
  
}
