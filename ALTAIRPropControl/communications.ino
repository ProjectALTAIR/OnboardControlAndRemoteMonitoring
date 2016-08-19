byte parse_serial_data( const byte* rx_data, byte rx_length, byte requestedAddress, struct UM7packet* packet ) {
  byte index;
  // Make sure that the data buffer provided is long enough to contain a full packet
  // The minimum packet length is 7 bytes
  if ( rx_length < 7 ) {
    return 1;
  }
  // Try to find the ‘snp’ start sequence for the packet, and ensure that either
  //   a) the packet has the requested address, or 2) that any address will do (if requestedAddress == 0)
  for ( index = 0; index < (rx_length - 2); index++ ) {
    // Check for ‘snp’. If found, immediately exit the loop
    if ( rx_data[index] == 's' && rx_data[index + 1] == 'n' && rx_data[index + 2] == 'p' &&
         ( requestedAddress == 0 || rx_data[index + 4] == requestedAddress ) ) {
      break;
    }
  }
  byte packet_index = index;
  // Check to see if the variable ‘packet_index’ is equal to (rx_length - 2). If it is, then the above
  // loop executed to completion and never found a packet header.
  if ( packet_index == (rx_length - 2) ) {
    return 2;
  }
  // If we get here, a packet header was found. Now check to see if we have enough room
  // left in the buffer to contain a full packet. Note that at this point, the variable ‘packet_index’
  // contains the location of the ‘s’ character in the buffer (the first byte in the header)
  if ( (rx_length - packet_index) < 7 ) {
    return 3;
  }
  // We’ve found a packet header, and there is enough space left in the buffer for at least
  // the smallest allowable packet length (7 bytes). Pull out the packet type byte to determine
  // the actual length of this packet
  byte PT = rx_data[packet_index + 3];
  // Do some bit-level manipulation to determine if the packet contains data and if it is a batch
  // We have to do this because the individual bits in the PT byte specify the contents of the
  // packet.
  byte packet_has_data = (PT >> 7) & 0x01; // Check bit 7 (HAS_DATA)
  byte packet_is_batch = (PT >> 6) & 0x01; // Check bit 6 (IS_BATCH)
  byte batch_length = (PT >> 2) & 0x0F; // Extract the batch length (bits 2 through 5)
  // Now finally figure out the actual packet length
  byte data_length = 0;
  if ( packet_has_data ) {
    if ( packet_is_batch ) {
      // Packet has data and is a batch. This means it contains ‘batch_length' registers, each
      // of which has a length of 4 bytes
      data_length = 4 * batch_length;
    } else { // Packet has data but is not a batch. This means it contains one register (4 bytes)
      data_length = 4;
    }
  } else { // Packet has no data
    data_length = 0;
  }
  // At this point, we know exactly how long the packet is. Now we can check to make sure
  // we have enough data for the full packet.
  if ( (rx_length - packet_index) < (data_length + 5) ) {
    return 3;
  }
  // If we get here, we know that we have a full packet in the buffer. All that remains is to pull
  // out the data and make sure the checksum is good.
  // Start by extracting all the data
  packet->Address = rx_data[packet_index + 4];
  packet->PT = PT;
  // Get the data bytes and compute the checksum all in one step
  packet->data_length = data_length;
  unsigned int computed_checksum = 's' + 'n' + 'p' + packet->PT + packet->Address;
  for ( index = 0; index < data_length; index++ ) {
    // Copy the data into the packet structure’s data array
    packet->data[index] = rx_data[packet_index + 5 + index];
    // Add the new byte to the checksum
    computed_checksum += packet->data[index];
  }
  // Now see if our computed checksum matches the received checksum
  // First extract the checksum from the packet
  unsigned int received_checksum = (rx_data[packet_index + 5 + data_length] << 8);
  received_checksum |= rx_data[packet_index + 6 + data_length];
  // Now check to see if they don’t match
  if ( received_checksum != computed_checksum ) {
    return 4;
  }
  // At this point, we’ve received a full packet with a good checksum. It is already
  // fully parsed and copied to the ‘packet’ structure, so return 0 to indicate that a packet was
  // processed.
  return 0;
}

void checkUM7Health() {
  if (Serial3.available()) {
    byte tx_data[7];
    byte rx_data[rx_read_length];
    int returnVal;
    struct UM7packet new_packet;
    tx_data[0] = 's';  // Send
    tx_data[1] = 'n';  // New
    tx_data[2] = 'p';  // Packet
    tx_data[3] = 0x00; // packet type byte
    tx_data[4] = 0x55; // address of DREG_HEALTH sensor health info register
    tx_data[5] = 0x01; // checksum high byte
    tx_data[6] = 0xA6; // checksum low byte
    Serial3.write( tx_data, 7 );
    Serial3.readBytes( rx_data, rx_read_length );
    returnVal  = parse_serial_data(rx_data, rx_read_length, tx_data[4], &new_packet);
    if ( returnVal == 0 ) {
      // Extract health info ...
      float sats_used_byte = -999., hdop_byte = -999., sats_in_view_byte = -999., sensors_byte = -999.;
      sats_used_byte    = new_packet.data[0];
      hdop_byte         = new_packet.data[1];
      sats_in_view_byte = new_packet.data[2];
      sensors_byte      = new_packet.data[3];
      // ... and print it out.
      Serial.print("sats_used_byte = "); Serial.print(sats_used_byte); Serial.print("    hdop_byte = "); Serial.print(hdop_byte);
      Serial.print("    sats_in_view_byte = "); Serial.print(sats_in_view_byte); Serial.print("    sensors_byte = "); Serial.println(sensors_byte);
    }
  }
}
void getDataFromUM7(){
  int yawInt = -999, pitchInt = -999, rollInt = -999;
  int yawRateInt = -999; int pitchRateInt = -999; int rollRateInt = -999;
  byte tx_data[7];
  byte rx_data[rx_read_length];
  int returnVal;
  struct UM7packet new_packet;
  tx_data[0] = 's';  // Send
  tx_data[1] = 'n';  // New
  tx_data[2] = 'p';  // Packet
  /* 
  tx_data[3] = 0x00; // packet type byte
  //    tx_data[4] = 0x70; // address of DREG_EULER_PHI_THETA roll and pitch angle info register
  tx_data[4] = 0x55; // address of DREG_HEALTH sensor health info register
  tx_data[5] = 0x01; // checksum high byte
  //    tx_data[6] = 0xC1; // checksum low byte
  tx_data[6] = 0xA6; // checksum low byte
  /*
      Serial3.write( tx_data, 7 );
      Serial3.readBytes( rx_data, rx_read_length );
      returnVal  = parse_serial_data(rx_data, rx_read_length, tx_data[4], &new_packet);
      if ( returnVal == 0 ) {
        // Extract the pitch and roll
    //      pitchInt = (new_packet.data[0] << 8) | new_packet.data[1];
    //      rollInt  = (new_packet.data[2] << 8) | new_packet.data[3];
    //      pitch    = pitchInt / 91.02222;
    //      roll     = rollInt  / 91.02222;
        // Extract health info ...
        float sats_used_byte = -999., hdop_byte = -999., sats_in_view_byte = -999., sensors_byte = -999.;
        sats_used_byte    = new_packet.data[0];
        hdop_byte         = new_packet.data[1];
        sats_in_view_byte = new_packet.data[2];
        sensors_byte      = new_packet.data[3];
        // ... and print it out.
        Serial.print("sats_used_byte = "); Serial.print(sats_used_byte); Serial.print("    hdop_byte = "); Serial.print(hdop_byte);
           Serial.print("    sats_in_view_byte = "); Serial.print(sats_in_view_byte); Serial.print("    sensors_byte = "); Serial.println(sensors_byte);
      }
  */
  tx_data[3] = 0x7C; // get a batch of 16 register words (= 64 bytes)
  tx_data[4] = 0x65; // start with the processed accelerometer info: address of DREG_ACCEL_PROC_X register
  tx_data[5] = 0x02; // checksum high byte
  tx_data[6] = 0x32; // checksum low byte
  Serial3.write( tx_data, 7 );
  Serial3.readBytes( rx_data, rx_read_length );
  returnVal  = parse_serial_data(rx_data, rx_read_length, tx_data[4], &new_packet);
  if ( returnVal == 0 ) {
    // Extract stuff.
    // Extract the linear accelerations along the x-, y-, and z-axes respectively
    accel[0] = convertBytesToFloat(new_packet.data);
    accel[1] = convertBytesToFloat(&(new_packet.data[4]));
    accel[2] = convertBytesToFloat(&(new_packet.data[8]));
    // Extract the pitch and roll
    rollInt  = (new_packet.data[44] << 8) |  new_packet.data[45];
    pitchInt = (new_packet.data[46] << 8) |  new_packet.data[47];
    roll     = rollInt  / 91.02222;
    pitch    = pitchInt / 91.02222;
    // Extract the yaw
    yawInt   = (new_packet.data[48] << 8) |  new_packet.data[49];
    yaw      = yawInt / 91.02222;
    // Extract the roll rate and pitch rate and yaw rate
    rollRateInt  = (new_packet.data[52] << 8) | new_packet.data[53];
    rollRate     = rollRateInt / 16.0;
    pitchRateInt = (new_packet.data[54] << 8) | new_packet.data[55];
    pitchRate    = pitchRateInt / 16.0;
    yawRateInt   = (new_packet.data[56] << 8) | new_packet.data[57];
    yawRate      = yawRateInt / 16.0;
  }
  else {
    //      Serial.print("returnVal = "); Serial.println(returnVal);
  }
  /*
        if ( returnVal == 0 || returnVal == 4 ) {
          float datalenInt = new_packet.data_length;
          if ( datalenInt != 0 && returnVal == 0 ) {
          float theChecksum = new_packet.Checksum;
          float addressInt = new_packet.Address;
          float ptInt = new_packet.PT;
          Serial.println();
          Serial.print("Address = "); Serial.println(addressInt);
          Serial.print("PT = "); Serial.println(ptInt);
          Serial.print("Checksum = "); Serial.println(theChecksum);
          Serial.print("data_length = "); Serial.println(datalenInt);
          Serial.print("returnVal = "); Serial.println(returnVal);
          Serial.println();
          }
        } else {
          Serial.print("bad returnVal = "); Serial.println(returnVal);
        }
    */
}

void sendALTAIRinfoLine(double rpm[4], float current[4]){
  for(int i = 0; i < 7; i++) {Serial.print(setting[i]);       Serial.print(" ");}
  Serial.print("     ");
  for(int i = 0; i < 4; i++) {Serial.print(rpm[i]);           Serial.print(" ");}
  Serial.print("     ");
  for(int i = 0; i < 4; i++) {Serial.print(current[i]);       Serial.print(" ");}
  Serial.print("     ");
  for(int i = 0; i < 8; i++) {Serial.print(tempInCelsius[i]); Serial.print(" ");}
  Serial.print("     ");
  for(int i = 0; i < 3; i++) {Serial.print(accel[i]);         Serial.print(" ");}
  Serial.print("     ");
  Serial.print(yaw);       Serial.print(" ");
  Serial.print(pitch);     Serial.print(" ");
  Serial.print(roll);      Serial.print("      ");
  Serial.print(rotAng);    Serial.print("      ");
  Serial.print(UM7health); Serial.print(" ");
  Serial.println(UM7temp);
}

void doInstruction(byte inputByte, boolean *thingsHaveChanged){
  byte channelToModify = 0;
  switch (inputByte) {
    case 'A' :
      channelToModify = 0;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'B' :
      channelToModify = 1;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'C' :
      channelToModify = 2;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'D' :
      channelToModify = 3;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'E' :
      channelToModify = 4;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'F' :
      channelToModify = 5;
      ++setting[channelToModify];
      if (setting[channelToModify] >= 10.) --setting[channelToModify];
      break;
    case 'a' :
      channelToModify = 0;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'b' :
      channelToModify = 1;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'c' :
      channelToModify = 2;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'd' :
      channelToModify = 3;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'e' :
      channelToModify = 4;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'f' :
      channelToModify = 5;
      --setting[channelToModify];
      if (setting[channelToModify] < 0.) ++setting[channelToModify];
      break;
    case 'x' :
      setting[5] = 7.;
      for (int i = 0; i < 5; ++i) setting[i] = 0.;
      break;
    default :
      *thingsHaveChanged = false;
      break;
  }
}
