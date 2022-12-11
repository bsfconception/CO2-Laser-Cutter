/**********************************************************************************************************/
/**********************************************************************************************************/

void StoreShortInEeprom( short Value, short Index )
{
  for(int j=0;j<2;j++)
  {
    EEPROM.write(Index++, (unsigned char)((Value >> (8*j)) & 0xFF) );
  }
}

short RetrieveShortFromEeprom( short Index )
{
short Value=0;
  for(int j=0;j<2;j++)
  {
    Value = Value | ((long)EEPROM.read(Index++) << (8*j));
  }
  return(Value);
}

void LoadCalibration( void )
{
int _storage_idx, i, j;
unsigned char c;

  _storage_idx = 0;
  c = EEPROM.read(_storage_idx++);

  if( c != 0x55 )
  {
    Serial.println( ";No Saved Data");
    g_Parameters.MaxPWM = 25;   // %
    g_Parameters.MaxTemp = 30;  // °
    g_Parameters.ZToFocus = 800; // 1/10 mm
//    g_Parameters.ZDelta = 3;   // mm
    g_Parameters.Size = 10;     // mm
    g_Parameters.Speed = 10;     // mm/s
    g_Parameters.PWM = 10;      // %
    g_Parameters.Mode = 0;
    g_Parameters.Step = 20;     // 1/10 mm
    g_Parameters.Duration = 100;  // ms
    g_Parameters.Thickness = 30;  // 1/10 mm
    return;
  }
  Serial.println( ";Retrieve Saved Data");

  g_Parameters.MaxPWM = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";MaxPWM:");
  Serial.println(g_Parameters.MaxPWM);
  _storage_idx += 2;
  g_Parameters.MaxTemp = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";MaxTemp:");
  Serial.println(g_Parameters.MaxTemp);
  _storage_idx += 2;
  g_Parameters.ZToFocus = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";ZToFocus:");
  Serial.println(g_Parameters.ZToFocus);
  _storage_idx += 2;
//  g_Parameters.ZDelta = RetrieveShortFromEeprom( _storage_idx );
  _storage_idx += 2;
  g_Parameters.Size = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Size:");
  Serial.println(g_Parameters.Size);
  _storage_idx += 2;
  g_Parameters.Speed = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Speed:");
  Serial.println(g_Parameters.Speed);
  _storage_idx += 2;
  g_Parameters.PWM = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";PWM:");
  Serial.println(g_Parameters.PWM );
  _storage_idx += 2;
  g_Parameters.Mode = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Mode:");
  Serial.println(g_Parameters.Mode);
  _storage_idx += 2;
  g_Parameters.Step = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Step:");
  Serial.println(g_Parameters.Step);
  _storage_idx += 2;
  g_Parameters.Duration = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Duration:");
  Serial.println(g_Parameters.Duration);
  _storage_idx += 2;
  g_Parameters.Thickness = RetrieveShortFromEeprom( _storage_idx );
  Serial.print(";Thickness:");
  Serial.println(g_Parameters.Thickness);
  
}

void SaveCalibration( void )
{
int _storage_idx, i, j;
unsigned char c;

  Serial.println( ";Save Data");

  _storage_idx = 0;
  EEPROM.write(_storage_idx++, 0x55);


  StoreShortInEeprom( g_Parameters.MaxPWM, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.MaxTemp, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.ZToFocus, _storage_idx );
  _storage_idx += 2;
//  StoreShortInEeprom( g_Parameters.ZDelta, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Size, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Speed, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.PWM, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Mode, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Step, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Duration, _storage_idx );
  _storage_idx += 2;
  StoreShortInEeprom( g_Parameters.Thickness, _storage_idx );

  
}


void StoreData( void )
{
  switch(g_Menu)
  {
    case 0:
      break;
    case 1:
      switch(g_Item)
      {
        case 0: //g_Parameters.PWM
          g_Parameters.PWM = atoi(Tmp);
          analogWrite(PWM_OUTPUT_PIN, (g_Parameters.PWM*256)/100);
          SaveCalibration();
          break;
        case 1: //g_Parameters.Thickness
          g_Parameters.Thickness = atoi(Tmp);
          SaveCalibration();
          break;
      }
      break;
    case 2:
      switch(g_Item)
      {
        case 0: //g_Parameters.Size
          g_Parameters.Size = atoi(Tmp);
          SaveCalibration();
          break;
        case 1: //g_Parameters.Speed
          g_Parameters.Speed = atoi(Tmp);
          SaveCalibration();
          break;
        case 2: //g_Parameters.Mode
          g_Parameters.Mode = atoi(Tmp);
          SaveCalibration();
          break;
        case 3: //g_Parameters.Step
          g_Parameters.Step = atoi(Tmp);
          SaveCalibration();
          break;
        case 4: //g_Parameters.Duration
          g_Parameters.Duration = atoi(Tmp);
          SaveCalibration();
          break;
      }
      break;
    case 3:
      switch(g_Item)
      {
        case 0: //g_Parameters.MaxPWM
          g_Parameters.MaxPWM = atoi(Tmp);
          SaveCalibration();
          break;
        case 1: //g_Parameters.ZToFocus
          g_Parameters.ZToFocus = atoi(Tmp);
          SaveCalibration();
          break;
      }    
      break;
  }
  
}
