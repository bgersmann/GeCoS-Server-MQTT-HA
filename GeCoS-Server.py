#!/usr/bin/env python3
# encoding=utf-8
"""
GeCoS-Server
Steuert I2C-basierte Ein-/Ausgangsmodule über MQTT mit Home-Assistant-Integration
"""

import smbus
import time
import sys
import logging
from datetime import datetime
import socket
import threading
import configparser
import os
import argparse
import json
from typing import Optional, List, Dict, Tuple, Set, Any

import paho.mqtt.client as mqtt

from Mux import multiplex

# Logging konfigurieren
logging.basicConfig(
    level=logging.WARNING,
    format='%(asctime)s.%(msecs)03d %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

#Bei jeder Aenderung hochzaehlen - wird beim Start geloggt, damit sichtbar ist,
#welcher Stand tatsaechlich installiert ist.
__version__ = "2026.08.16-ow4"

# Status Variable 16IN 1x pro Bus mit 8 Werten
stat_in = {
    0: [0] * 8,
    1: [0] * 8,
    2: [0] * 8
}

# Globale Variablen
print_debug = False
status_ow = 1
mqtt_client: Optional[mqtt.Client] = None
mqtt_connected = threading.Event()
mqtt_topics: Dict[str, str] = {}
mqtt_settings: Dict[str, Any] = {}
input_state_cache: Dict[Tuple[int, int], int] = {}
output_state_cache: Dict[Tuple[int, int], int] = {}
pwm_state_cache: Dict[Tuple[int, int, int], Tuple[int, int]] = {}
analog_state_cache: Dict[Tuple[int, int, int], float] = {}
ow_state_cache: Dict[str, Any] = {}
ow_avail_cache: Dict[str, bool] = {}
ow_lock = threading.RLock()
ha_discovery_topics: Set[str] = set()

# Module-Arrays pro Kanal ('ow' haengt am DS2482 und kennt keine Kanaele)
modules = {
    'in': {0: [], 1: [], 2: []},
    'out': {0: [], 1: [], 2: []},
    'pwm': {0: [], 1: [], 2: []},
    'rgbw': {0: [], 1: [], 2: []},
    'ana': {0: [], 1: [], 2: []},
    'ow': []
}

I2C_ADR_DS2482 = 0x18  # Adresse DS2482

# OneWire Family-Codes
OW_FAMILY_TEMPERATURE = ("28", "10", "3b")  # DS18B20, DS18S20, MAX31850
OW_FAMILY_SWITCH = "3a"                     # DS2413


def _ow_address_string(device_address) -> str:
    """Formatiert eine OneWire-ROM-Adresse als '<family>-<crc+serie>'"""
    raw = ((device_address[0] << 32) | (device_address[1] & 0xFFFFFFFF)) & 0xFFFFFFFFFFFFFFFF
    hexstr = f"{raw:016x}"
    return f"{hexstr[-2:]}-{hexstr[:14]}"

# DS2482 - OneWire-Bridge-Klasse
class DS2482:
    """
    DS2482 OneWire-zu-I2C-Bridge Klasse
    
    Ermöglicht die Kommunikation mit OneWire-Geräten über I2C
    """
    
    I2C_ADDR = I2C_ADR_DS2482
    
    def __init__(self):
        self._bus = smbus.SMBus(bus)
        self._owDeviceAddress = [0, 0]
        self._owTripletDirection = 1
        self._owTripletFirstBit = 0
        self._owTripletSecondBit = 0
        self._owLastDevice = 0
        self._owLastDiscrepancy = 0
        self._owLastStatus = None
        try:
            #Geraete-Reset nach dem Einschalten, sonst kann der erste Bus-Reset scheitern
            self.DS2482Reset()
            time.sleep(0.001)
            self.DS2482SetConfig(mqtt_settings.get("ow_active_pullup", True))
        except Exception as e:
            log(f"DS2482 auf {hex(self.I2C_ADDR)} nicht erreichbar: {e}", "ERROR")

    def DS2482SetConfig(self, apu: bool = True) -> bool:
        """
        Schreibt das Konfigurationsregister (0xD2)

        Args:
            apu: Aktiver Pull-up. Ohne externen 4,7k-Pull-up zwingend noetig,
                 sonst erreicht die 1Wire-Leitung nie sauber High-Pegel.
        """
        try:
            config = 0x01 if apu else 0x00
            #Unteres Nibble = Konfiguration, oberes Nibble = deren Einerkomplement
            self._bus.write_byte_data(self.I2C_ADDR, 0xD2, ((~config & 0x0F) << 4) | config)
            log("DS2482 Konfiguration gesetzt (aktiver Pull-up: {0})".format("ein" if apu else "aus"), "INFO")
            return True
        except Exception as e:
            log(f"DS2482 Konfiguration fehlgeschlagen: {e}", "ERROR")
            return False

    def OWStatusText(self) -> str:
        """Uebersetzt das zuletzt gelesene Statusregister in Klartext"""
        data = self._owLastStatus
        if data is None:
            return "DS2482 antwortet nicht ueber I2C"
        if data & 0x01:
            return f"1Wire-Bus dauerhaft belegt (Status 0x{data:02x})"
        if data & 0x04:
            return f"Kurzschluss auf der 1Wire-Leitung (Status 0x{data:02x})"
        if not (data & 0x02):
            return (f"kein Presence-Pulse (Status 0x{data:02x}) - kein Sensor angeschlossen, "
                    "fehlender Pull-up oder keine Versorgung am Strang")
        return f"Status 0x{data:02x}"

    def OWSearchBus(self):
        """
        Sucht alle OneWire-Geräte auf dem Bus

        Returns:
            Optional[List[str]]: Gefundene Adressen, None bei Abbruch
        """
        found: List[str] = []
        try:
            while self.OWSearch() == 1:
                device = _ow_address_string(self._owDeviceAddress)
                found.append(device)
                publish_command_event("OWS", device=device)
                log("Gerät gefunden: " + str(device), "INFO")
            publish_command_event("OWS", status="END")
            return found
        except Exception as e:
            publish_command_event("OWS", status="ERROR", message=str(e))
            log(f"Fehler bei OWS Suche: {e}", "ERROR")
            return None

    def DS2482Reset(self):
        """Setzt den DS2482 zurück"""
        self._bus.write_byte(self.I2C_ADDR, 0xF0)

    def OWStatusRegister(self):
        """Liest das Status-Register"""
        self._bus.write_byte_data(self.I2C_ADDR, 0xE1, 0xF0)
        e = self._bus.read_byte(self.I2C_ADDR)
        return e

    def OWReset(self):
        """Setzt den OneWire-Bus zurück"""
        self._bus.write_byte(self.I2C_ADDR, 0xB4)  # 1Wire Reset
        loopcount=0
        data=""    
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            self._owLastStatus = data
            if (data is None):
                #Fehler beim Lesen
                return 0
            else:
                if (data & 0x01):
                    #1Wire belegt
                    if (loopcount>100):
                        return 0
                else:
                    if (data & 0x04):
                        #Short detect bit
                        return 0
                    if (data & 0x02):
                        #Presense-Pulse Detect bit
                        break
                    else:
                        #Keine OW geräte gefunden
                        return 0
        return 1


    def OWWriteByte(self,byte):
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xF0)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte_data(self.I2C_ADDR,0xA5, byte)
        loopcount=0
        while (True):
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break   
        return 0


    def OWReadByte(self):
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xF0)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte(self.I2C_ADDR,0x96)
        loopcount=0
        while (True):
            loopcount+=1
            data=self.OWStatusRegister()
            if(data is None):
                #Fehler
                return -1
            else:
                if (data & 0x01):
                    if loopcount>100:
                        #Fehler I2C Belegt
                        return -1
                    time.sleep(0.001)
                else:
                    break
        self._bus.write_byte_data(self.I2C_ADDR,0xE1,0xE1)
        data=self._bus.read_byte(self.I2C_ADDR)
        if(data is None):
                #Fehler
                return -1
        return data
                



    
    def OWTriplet(self):
        if (self._owTripletDirection > 0):
            self._owTripletDirection = 0xFF
        self._bus.write_byte_data(self.I2C_ADDR, 0x78,self._owTripletDirection)
        loopcount = 0
        while (True):
            loopcount+=1
            data =self.OWStatusRegister()
            if (data is None):
                return -1
            else:
                if (data & 0x01):
                    if (loopcount > 100):
                        return -1
                else:
                    if (data & 0x20):
                        self._owTripletFirstBit = 1
                    else:
                        self._owTripletFirstBit = 0
                    if (data & 0x40):
                        self._owTripletSecondBit = 1
                    else:
                        self._owTripletSecondBit = 0
                    if (data & 0x80):
                        self._owTripletDirection = 1
                    else:
                        self._owTripletDirection = 0
                    return 1


    def OWSearch(self):
        #global owDeviceAddress
        self._bitNumber=1
        self._lastZero=0
        self._deviceAddress4ByteIndex=1 #Fill last 4 bytes first, data from onewire comes LSB first.
        self._deviceAddress4ByteMask=1
        
        if (self._owLastDevice):
            #Letzte adresse:
            self._owLastDevice=0
            self._owLastDiscrepancy=0
            self._owDeviceAddress[0] = 0xFFFFFFFF
            self._owDeviceAddress[1] = 0xFFFFFFFF
        else:
            if not (self.OWReset()):
                self._owLastDiscrepancy = 0
                return 0
        
            self.OWWriteByte(0xF0)
            while (self._deviceAddress4ByteIndex > -1):
                if (self._bitNumber < self._owLastDiscrepancy):
                    if (self._owDeviceAddress[self._deviceAddress4ByteIndex] & self._deviceAddress4ByteMask):
                        self._owTripletDirection = 1
                    else:
                        self._owTripletDirection = 0
                elif (self._bitNumber == self._owLastDiscrepancy): #if equal to last pick 1, if not pick 0
                    self._owTripletDirection = 1
                else:
                    self._owTripletDirection = 0
                
                if not (self.OWTriplet()):
                    return 0

                if (self._owTripletFirstBit==0 and self._owTripletSecondBit==0 and self._owTripletDirection==0):
                    self._lastZero = self._bitNumber
                if (self._owTripletFirstBit==1 and self._owTripletSecondBit==1):
                    break
                if (self._owTripletDirection==1):
                    self._owDeviceAddress[self._deviceAddress4ByteIndex] = self._owDeviceAddress[self._deviceAddress4ByteIndex] | self._deviceAddress4ByteMask
                else:
                    self._owDeviceAddress[self._deviceAddress4ByteIndex] = self._owDeviceAddress[self._deviceAddress4ByteIndex] & (~self._deviceAddress4ByteMask)
                self._bitNumber+=1 #Counter hochsetzen
                self._deviceAddress4ByteMask = (self._deviceAddress4ByteMask << 1) & 0xFFFFFFFF #shift the bit mask left
                if (self._deviceAddress4ByteMask == 0): #if the mask is 0 then go to other address block and reset mask to first bit
                    self._deviceAddress4ByteIndex=self._deviceAddress4ByteIndex-1
                    self._deviceAddress4ByteMask = 1

            if (self._bitNumber == 65): #if the search was successful then
                self._owLastDiscrepancy = self._lastZero
                if (self._owLastDiscrepancy==0):
                    self._owLastDevice = 1
                else:
                    self._owLastDevice = 0
                
                #serialnumber=owDeviceAddress[0][0]<<32 | owDeviceAddress[0][1]
                if (self.OWCheckCRC()):
                    #CRC OK
                    return 1
                else:
                    #CRC NICHT OK
                    return 0
        self._owLastDiscrepancy = 0
        self._owLastDevice = 0
        return 0


    def OWCheckCRC(self):
        crc = 0
        da32bit= self._owDeviceAddress[1]
        for j in range(0,4):
            crc = self.AddCRC(da32bit & 0xFF, crc)
            da32bit = da32bit >> 8 #Shift right 8 bits
        da32bit = self._owDeviceAddress[0]
        for j in range(0,3):
            crc = self.AddCRC(da32bit & 0xFF, crc)
            da32bit = da32bit >> 8 #Shift right 8 bits
        if ((da32bit & 0xFF) == crc): #last byte of address should match CRC of other 7 bytes
            return 1 #match
        return 0 #bad CRC


    def AddCRC(self,inbyte, crc):
        for j in range(0,8):
            mix = (crc ^ inbyte) & 0x01
            crc = crc >> 1
            if (mix):
                crc = crc ^ 0x8C
            inbyte = inbyte >> 1
        return crc


    def OWSelect(self):
        self.OWWriteByte(0x55) #Issue the Match ROM command
        #for i in range(1,-1,-1):
        da32bit = self._owDeviceAddress[1]
        for j in range(0,4):
            self.OWWriteByte(da32bit & 0xFF) #Send lowest byte
            da32bit = da32bit >> 8 #Shift right 8 bits
        da32bit = self._owDeviceAddress[0]
        for j2 in range(0,4):
            self.OWWriteByte(da32bit & 0xFF) #Send lowest byte
            da32bit = da32bit >> 8 #Shift right 8 bits

    def OWSelectAdress(self,OWAdr):
        #"28-a601183074cbff" -> a601183074cbff28
        try:
            x = OWAdr.split("-")
            tmp2 = "0x" + x[1] + x[0]
            tmp = int(tmp2, 16)
            self._owDeviceAddress[1] = tmp & 0xFFFFFFFF
            self._owDeviceAddress[0] = tmp >> 32
            return True
        except (ValueError, IndexError) as e:
            log(f"Fehler beim OneWire Adresse einstellen: {e}", "ERROR")
            return False

    def DS2413OWSetConfig(self,data):
        try:
            if not (self.OWReset()): #Match ROM benoetigt vorher einen Bus-Reset
                return False
            self.OWSelect()
            self.OWWriteByte(0x5a)
            self.OWWriteByte(data)
            data = ~data&0xFF
            self.OWWriteByte(data)
            return True
        except Exception as e:
            log(f"Fehler beim DS2413 Config setzen: {e}", "ERROR")
            return False

    def DS18B20OWSetConfig(self,res):
        try:
            # 31, 63, 95, 127 9/10/11/12Bit
            if not (self.OWReset()): #Match ROM benoetigt vorher einen Bus-Reset
                return False
            self.OWSelect()
            self.OWWriteByte(78)
            self.OWWriteByte(0)
            self.OWWriteByte(0)
            self.OWWriteByte(res)
            return True
        except Exception as e:
            log(f"Fehler beim DS18B20 Config setzen: {e}", "ERROR")
            return False
        

    def DS18B20OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x28): #Ist ein DS18B20
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.760) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte

            data = [0,0,0,0,0,0,0,0,0]
            for i in range(0,9):                
                data[i] = self.OWReadByte()
               
            crc = 0
            for j in range(0,8):
                crc = self.AddCRC(data[j], crc)
            
            if data[8] != crc:
                 celsius=-85
                 return celsius

            raw = (data[1] << 8) | data[0]
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment
            cfg = data[4] & 0x60
            if (cfg == 0x60):
                raw=raw
                #nix tun
            elif (cfg == 0x40):
                #raw = raw & 0xFFFE
                raw = raw << 1
            elif (cfg == 0x20):
                #raw = raw & 0xFFFC
                raw = raw << 2
            else:
                #raw = raw & 0xFFF8
                raw = raw << 3

            celsius = raw / 16.0
            if (SignBit):
                celsius = celsius * (-1)
            device=_ow_address_string(self._owDeviceAddress)
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=_ow_address_string(self._owDeviceAddress)
            log(f"Fehler 1Wire DS18B20 {device}: {e}","ERROR")
        finally:
            return celsius

    def DS2413GetState(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x3a): #Ist ein DS2413
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0xF5) #Starte Messung
                    result = self.OWReadByte()
        except Exception as e:
            result=-85
            device=_ow_address_string(self._owDeviceAddress)
            log(f"Fehler 1Wire DS2413 {device}: {e}","ERROR")
        finally:
            return result


    def MAX31850OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x3B): #Ist ein MAX31850
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.100) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte

            data = [0,0,0,0]
            for i in range(0,4):
                data[i] = self.OWReadByte()

            raw = (data[1] << 8) | data[0] & 0xFC
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment

            if (data[0]&0X01==1): # Auf fehler prüfen
                celsius=-85
                device=_ow_address_string(self._owDeviceAddress)
                log("Device: " + str(device) + " Temp: " + str(celsius),"ERROR")
                return celsius

            celsius= raw * 0.0625
            device=_ow_address_string(self._owDeviceAddress)
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=_ow_address_string(self._owDeviceAddress)
            log(f"Fehler 1Wire MAX31850 {device}: {e}","ERROR")
        finally:
            return celsius



    def DS18S20OWReadTemp(self):
        try:
            if ((self._owDeviceAddress[1]& 0xFF) == 0x10): #Ist ein DS18S20
                if (self.OWReset()):
                    self.OWSelect()
                    self.OWWriteByte(0x44) # Starte Messung
                    time.sleep(0.750) #Warten auf messung
                    if (self.OWReset()):
                        self.OWSelect()
                        self.OWWriteByte(0xBE) #Lese Werte


            data = [0,0]
            for i in range(0,2):
                data[i] = self.OWReadByte()

            raw = (data[1] << 8) | data[0]
            SignBit = raw & 0x8000  # test most significant bit
            if (SignBit):
                raw = (raw ^ 0xffff) + 1 # negative, 2's compliment
            
            celsius = raw / 2.0
            if (SignBit):
                celsius = celsius * (-1)
            device=_ow_address_string(self._owDeviceAddress)
            #log("Device: " + str(device) + " Temp: " + str(celsius),"INFO")
        except Exception as e:
            celsius=-85
            device=_ow_address_string(self._owDeviceAddress)
            log(f"Fehler 1Wire DS18S20 {device}: {e}","ERROR")
        finally:
            return celsius


#RTC:
def _bcd_to_int(x):
    # Decode 2x4 bit BCD to byte value
    return int((x//16)*10 + x%16)

def _int_to_bcd(x):
    # Encode byte value to BCD
    return int((x//10)*16 + x%10)

#http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-RTC/DS1307_lib.py
class DS1307():
    DS_REG_SECONDS = 0x00
    DS_REG_MINUTES = 0x01
    DS_REG_HOURS   = 0x02
    DS_REG_DOW     = 0x03
    DS_REG_DAY     = 0x04
    DS_REG_MONTH   = 0x05
    DS_REG_YEAR    = 0x06
    DS_REG_CONTROL = 0x07
    DS_REG_TEMP_HSB = 0x11
    DS_REG_TEMP_LSB = 0x12
    

    def __init__(self, bus, addr=0x68):
        self._bus = bus #smbus.SMBus(twi)
        self._addr = addr

    def _read_seconds(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_SECONDS))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_SECONDS))
    
    def _read_minutes(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_MINUTES))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_MINUTES))

    def _read_hours(self):
        d = self._bus.readByteData(3,self._addr, self.DS_REG_HOURS)
        #d = self._bus.read_byte_data(self._addr, self.DS_REG_HOURS)
        if (d == 0x64):    # 12-Std.-Modus
            if ((d & 0b00100000) > 0):
                # Umrechnen auf 24-Std.-Modus
                return _bcd_to_int(d & 0x3F) + 12
        return _bcd_to_int(d & 0x3F)

    def _read_dow(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_DOW))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_DOW))

    def _read_day(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_DAY))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_DAY))

    def _read_month(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_MONTH)&0b01111111)
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_MONTH)&0b01111111)

    def _read_year(self):
        return _bcd_to_int(self._bus.readByteData(3,self._addr, self.DS_REG_YEAR))
        #return _bcd_to_int(self._bus.read_byte_data(self._addr, self.DS_REG_YEAR))

    def read_temp(self):
        byte_tmsb = self._bus.readByteData(3,self._addr,self.DS_REG_TEMP_HSB)
        byte_tlsb = bin(self._bus.readByteData(3,self._addr,self.DS_REG_TEMP_LSB))[2:].zfill(8)
        # byte_tmsb = self._bus.read_byte_data(self._addr,self.DS_REG_TEMP_HSB)
        # byte_tlsb = bin(self._bus.read_byte_data(self._addr,self.DS_REG_TEMP_LSB))[2:].zfill(8)
        return byte_tmsb+int(byte_tlsb[0])*2**(-1)+int(byte_tlsb[1])*2**(-2)

    def read_all(self):
        # Gibt eine Liste zurueck: (year, month, day, dow, hours, minutes, seconds).
        return (self._read_year(), self._read_month(), self._read_day(),
               self._read_dow(), self._read_hours(), self._read_minutes(),
               self._read_seconds())

    def read_str(self, century=20):
        # Gibt einen Datum/Zeit-String im Format 'YYYY-DD-MM HH:MM:SS' zurueck.
        return '%04d-%02d-%02d %02d:%02d:%02d' % (century*100 + self._read_year(),
               self._read_month(), self._read_day(), self._read_hours(),
               self._read_minutes(), self._read_seconds())

    def read_datetime(self, century=20, tzinfo=None):
        # Gibt ein datetime.datetime Objekt zurueck.
        return datetime(century*100 + self._read_year(),
               self._read_month(), self._read_day(), self._read_hours(),
               self._read_minutes(), self._read_seconds(), 0, tzinfo=tzinfo)

    def set_clock(self, century=20):
        # Liest einen Datum/Zeit-String im Format 'MMDDhhmmYYss' aus der RTC 
        # und setzt das Systemdatum mittels date-Kommando.
        cmd = 'sudo date %02d%02d%02d%02d%04d.%02d' % (self._read_month(),
              self._read_day(), self._read_hours(), self._read_minutes(),
              century*100 + self._read_year(), self._read_seconds())
        os.system(cmd)


    def write_all(self, seconds=None, minutes=None, hours=None, dow=None,
                  day=None, month=None, year=None):
        # Setzt Datum und Uhrzeit der RTC, jedoch nur die nicht-None-Werte.
        # Prueft auf Einhaltung der zulaessigen Wertegrenzen:
        #        seconds [0-59], minutes [0-59], hours [0-23],
        #        dow [1-7], day [1-31], month [1-12], year [0-99].
        if seconds is not None:
            if seconds < 0 or seconds > 59:
                raise ValueError('Seconds out of range [0-59].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_SECONDS, _int_to_bcd(seconds))

        if minutes is not None:
            if minutes < 0 or minutes > 59:
                raise ValueError('Minutes out of range [0-59].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_MINUTES, _int_to_bcd(minutes))

        if hours is not None:
            if hours < 0 or hours > 23:
                raise ValueError('Hours out of range [0-23].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_HOURS, _int_to_bcd(hours))

        if year is not None:
            if year < 0 or year > 99:
                raise ValueError('Year out of range [0-99].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_YEAR, _int_to_bcd(year))

        if month is not None:
            if month < 1 or month > 12:
                raise ValueError('Month out of range [1-12].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_MONTH, _int_to_bcd(month))

        if day is not None:
            if day < 1 or day > 31:
                raise ValueError('Day out of range [1-31].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_DAY, _int_to_bcd(day))

        if dow is not None:
            if dow < 1 or dow > 7:
                raise ValueError('DOW out of range [1-7].')
            self._bus.writeByteData(3,self._addr, self.DS_REG_DOW, _int_to_bcd(dow))

    def write_datetime(self, dto):
        # Setzt Datum/Zeit der RTC aus dem Inhalt eines datetime.datetime-Objekts.
        # isoweekday() liefert: Montag = 1, Dienstag = 2, ..., Sonntag = 7;
        # RTC braucht: Sonntag = 1, Montag = 2, ..., Samstag = 7
        wd = dto.isoweekday() + 1 # 1..7 -> 2..8
        if wd == 8:               # Sonntag
            wd = 1
        self.write_all(dto.second, dto.minute, dto.hour, wd,
                       dto.day, dto.month, dto.year % 100)

    def write_now(self):
        # Aequivalent zu write_datetime(datetime.datetime.now()).
        self.write_datetime(datetime.now())


#Konfiguration schreiben, wenn nicht vorhanden, anlegen, sonst gewünschte Daten hinzufügen/Anpassen
def configSchreiben(bereich,wert1, wert2):
    config = configparser.ConfigParser()
    config.read('Config.cfg')
    if config.has_section('Allgemein') != True:
        config['Allgemein'] = {'IP':'127.0.0.1','Port':'8000',
                            'StartZeit':str(datetime.now())}
        
    if bereich=='Allgemein':       
        if config.has_option('Allgemein','StartZeit'):
            config.set('Allgemein','StartZeit',str(datetime.now()))
        else:                    
            config['Allgemein'] = {'StartZeit':str(datetime.now())}
    else:
        if config.has_section(bereich):
            if config.has_option(bereich,wert1):
                config.set(bereich,wert1,wert2)
            else:
                config[bereich][wert1] = wert2
        else:
            config.add_section(bereich)
            if config.has_option(bereich,wert1):
                config.set(bereich,wert1,wert2)
            else:
                config[bereich][wert1] = wert2                    
    with open('Config.cfg','w') as configfile:
        config.write(configfile)
        configfile.close

def _check_OW() -> bool:
    """
    Prüft den OneWire-Bus-Status mit Timeout
    
    Returns:
        bool: True wenn verfügbar, sonst False
    """
    global status_ow
    i_cnt = 0
    while True:
        if status_ow == 1:
            return True
        else:
            i_cnt += 1
            if i_cnt >= 10000:
                log(f"OW Status: {status_ow}", "ERROR")
                return False
            time.sleep(0.001)
    return False

def set_output_konfig(kanal,adresse) -> bool:
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig","ERROR")
        return False

    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return False
    #Konfiguration als Ausgangsmodul:
    try:
        ergebnis = [
            plexer.writeByteData(kanal,adresse,bankAKonfig,outputKonfig),
            plexer.writeByteData(kanal,adresse,bankBKonfig,outputKonfig)
        ]
        if not all(ergebnis):
            log(f"Output-Konfiguration unvollstaendig (Kanal {kanal}, Addr {hex(adresse)})","ERROR")
            return False
        log("Adresse: " +str(hex(adresse)) + " - Port A + B als Output gesetzt")
        return True
    except Exception as e:
        log(f"Fehler beim Output konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")
        return False

def set_pwm_konfig(kanal, adresse) -> bool:
    if adresse <0x50 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        return False

    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return False
    try:
        #prescale: round((25.000.000/(4096*Freuqnz))-1) Frequenz aus Konfig lesen!
        prescale=round((25000000/(4096*freqStd))-1)
        ergebnis = [
            #Mode1 = sleep  Register 0  Wert = 16
            plexer.writeByteData(kanal,adresse,0x00,0x10),
            plexer.writeByteData(kanal,adresse,0xFE,prescale),
            #mode1 = sleep Register 0  Wert=32
            plexer.writeByteData(kanal,adresse,0x00,0x20),
            #mode2 = Ausgang Register 1  Wert = 4
            plexer.writeByteData(kanal,adresse,0x01,0x04)
        ]
        if not all(ergebnis):
            log(f"PWM-Konfiguration unvollstaendig (Kanal {kanal}, Addr {hex(adresse)})","ERROR")
            return False
        log("Adresse: {0} - PWM Konfig gesetzt".format(hex(adresse)))
        return True
    except Exception as e:
        log(f"Fehler beim PWM konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")
        return False
    
def set_input_konfig(kanal,adresse) -> bool:
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig","ERROR")
        return False

    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        return False
    #Konfiguration als Eingangsmodul:
    try:
        register = [
            (bankAKonfig, inputKonfig), (bankBKonfig, inputKonfig),
            (IOCONA, 0x44), (IOCONB, 0x44),
            (DEFVALA, 0x00), (DEFVALB, 0x00),
            (INTCONA, 0x00), (INTCONB, 0x00),
            (GPPUA, 0x00), (GPPUB, 0x00),
            (IPOLA, 0x00), (IPOLB, 0x00),
            (GPINTENA, 0xFF), (GPINTENB, 0xFF)
        ]
        ergebnis = [plexer.writeByteData(kanal,adresse,reg,wert) for reg, wert in register]
        if not all(ergebnis):
            log(f"Input-Konfiguration unvollstaendig (Kanal {kanal}, Addr {hex(adresse)})","ERROR")
            return False
        log("Adresse:{0} - Port A + B als Input gesetzt".format(hex(adresse)),"INFO")
        return True
    except Exception as e:
        log(f"Fehler beim Input konfigurieren (Kanal {kanal}, Addr {hex(adresse)}): {e}","ERROR")
        return False


def _ow_read_raw(address: str):
    """
    Liest einen OneWire-Sensor am DS2482

    Returns:
        Tuple[Any, str]: (Wert, Status). Der Wert -85 signalisiert einen Fehler.
    """
    global status_ow
    family = address.split("-")[0].lower()
    if family not in OW_FAMILY_TEMPERATURE and family != OW_FAMILY_SWITCH:
        log("OneWire Typ nicht unterstützt", "INFO")
        return -85, "Typ nicht untersützt"
    with ow_lock:
        if not _check_OW():
            return -85, "Fehler OW Bus belegt"
        status_ow = 0
        try:
            if dsOW.OWSelectAdress(address) != True:
                log("Fehler beim Adresse einstellen", "INFO")
                return -85, "Fehler bei Adresse einstellen"
            if family == "28":
                return dsOW.DS18B20OWReadTemp(), "OK"
            if family == "10":
                return dsOW.DS18S20OWReadTemp(), "OK"
            if family == "3b":
                return dsOW.MAX31850OWReadTemp(), "OK"
            return dsOW.DS2413GetState(), "OK"
        finally:
            status_ow = 1


def _ow_write_switch(address: str, value: int) -> str:
    """Schreibt das PIO-Latch-Byte eines DS2413 und liefert den Status zurueck"""
    global status_ow
    with ow_lock:
        if not _check_OW():
            return "Fehler OW Bus belegt"
        status_ow = 0
        try:
            if dsOW.OWSelectAdress(address) != True:
                log("Fehler beim Adresse einstellen", "INFO")
                return "Fehlerhafte OW Adresse"
            return "OK" if dsOW.DS2413OWSetConfig(value) == True else "FEHLER"
        finally:
            status_ow = 1


def OWReadDevice(arr):
    """Liest Daten von OneWire-Geräten und veröffentlicht sie über MQTT"""
    address = arr[1]
    value, status = _ow_read_raw(address)
    publish_ow_state(address, value, status)
    publish_command_event("OWV", address=address, value=value, status=status)


def OWConfigDevice(arr):
    """Konfiguriert OneWire-Geräte"""
    global status_ow
    address = arr[1]
    family = address.split("-")[0].lower()
    status = ""

    if family == "28":
        with ow_lock:
            if _check_OW():
                status_ow = 0
                if dsOW.OWSelectAdress(address) == True:
                    if dsOW.DS18B20OWSetConfig(int(arr[2])) == True:
                        status = "OK"
                    else:
                        status = "FEHLER"
                else:
                    log("Fehler beim Adresse einstellen", "INFO")
                    status = "Fehlerhafte OW Adresse"
                status_ow = 1
            else:
                status = "Fehler OW Bus belegt"
    elif family == OW_FAMILY_SWITCH:
        status = _ow_write_switch(address, int(arr[2]))
    else:
        log("OneWire Typ nicht unterstützt", "INFO")
        status = "Typ nicht untersützt"

    publish_command_event("OWC", address=address, value=arr[2] if len(arr) > 2 else None, status=status)
    if family == OW_FAMILY_SWITCH and status == "OK":
        #Ist-Zustand nachlesen, damit Home Assistant den Schaltvorgang bestaetigt bekommt
        value, read_status = _ow_read_raw(address)
        publish_ow_state(address, value, read_status)


def OWSetSwitch(address: str, pin: str, state: bool) -> None:
    """Schaltet einen einzelnen DS2413-Ausgang, ohne den zweiten zu veraendern"""
    cached = ow_state_cache.get(address)
    if cached is None:
        value, status = _ow_read_raw(address)
        publish_ow_state(address, value, status)
        cached = ow_state_cache.get(address)
        if cached is None:
            log(f"OneWire Zustand von {address} unbekannt, Schaltbefehl verworfen", "ERROR")
            return
    pin_a = _ow_latch_is_on(cached, "a")
    pin_b = _ow_latch_is_on(cached, "b")
    if pin == "a":
        pin_a = state
    else:
        pin_b = state
    #Open-Drain: Latch-Bit 0 = Ausgang aktiv, Bits 2-7 laut Datenblatt auf 1
    value = 0xFC
    if not pin_a:
        value |= 0x01
    if not pin_b:
        value |= 0x02
    OWConfigDevice(["OWC", address, str(value)])


def OWSearchDevice():
    """Sucht nach OneWire-Geräten auf dem Bus und meldet sie bei Home Assistant an"""
    global status_ow
    with ow_lock:
        if not _check_OW():
            log("OneWire Bus Belegt", "INFO")
            return
        status_ow = 0
        try:
            found = dsOW.OWSearchBus()
        finally:
            status_ow = 1
    if found is None:
        #Abgebrochene Suche darf die bekannten Geräte nicht verwerfen
        return
    for address in list(ow_state_cache):
        if address not in found:
            ow_state_cache.pop(address, None)
            ow_avail_cache.pop(address, None)
    modules['ow'].clear()
    modules['ow'].extend(found)
    configSchreiben('Module OneWire', 'GECOSOW', "".join(f"{a};" for a in found))
    if found:
        log("OneWire Geräte gefunden: {0}".format(len(found)), "INFO")
    else:
        log("OneWire Bus leer: {0}".format(dsOW.OWStatusText()), "WARNING")
    publish_ha_discovery()
    threading.Thread(target=ow_read_all, daemon=True).start()


def ow_read_all() -> None:
    """Liest alle bekannten OneWire-Geräte einmal aus"""
    for address in list(modules['ow']):
        try:
            value, status = _ow_read_raw(address)
            publish_ow_state(address, value, status)
        except Exception as exc:
            log(f"Fehler beim OneWire Lesen {address}: {exc}", "ERROR")


def ow_poll_loop() -> None:
    """Zyklisches Auslesen aller OneWire-Geräte"""
    interval = mqtt_settings.get("ow_interval", 0)
    if interval <= 0:
        return
    while True:
        time.sleep(interval)
        ow_read_all()


def thread_interrupt(pin):
    threading.Thread(target=interrutpKanal, args=(pin,), daemon=True).start()

def thread_OW_read(arr):
    threading.Thread(target=OWReadDevice, args=(arr,), daemon=True).start()

def thread_OW_config(arr):
    threading.Thread(target=OWConfigDevice, args=(arr,), daemon=True).start()

def thread_OW_Search():
    threading.Thread(target=OWSearchDevice, daemon=True).start()

def thread_OW_switch(address, pin, state):
    threading.Thread(target=OWSetSwitch, args=(address, pin, state), daemon=True).start()

def read_output(kanal,adresse):
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        publish_command_event("SAO", channel=kanal, address=hex(adresse), status="Modul adresse ungueltig")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        publish_command_event("SAO", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return

    value = None
    try:
        #Bytes fuer Bank A + B auslesen
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        iOut = [iOutB, iOutA]
        value=int.from_bytes(iOut,"big")
        sStatus="OK"
        publish_output_bits(kanal, adresse, value)
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler Output lesen"
        log(f"Fehler Output lesen (Kanal {kanal}, Addr {hex(adresse)})", "ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        sStatus=sStatus.replace(";","")
        publish_command_event("SAO", channel=kanal, address=hex(adresse), value=value, status=sStatus)

def set_output(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    value=int(arr[3]) if len(arr) > 3 else None
    if adresse <0x24 or adresse > 0x27:
        log("Modul adresse ungueltig: {0}".format(adresse))
        publish_command_event("SOM", channel=kanal, address=hex(adresse), value=value, status="Modul adresse ungueltig")
        return
        
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig")
        publish_command_event("SOM", channel=kanal, address=hex(adresse), value=value, status="Kanal ungueltig")
        return
    status=""
    try:
        #Bytes fuer Bank A + B auslesen
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        tmpArrOut=value.to_bytes(2,"big") if value is not None else (0).to_bytes(2,"big")
        iOutA=tmpArrOut[1]
        iOutB=tmpArrOut[0]
        plexer.writeByteData(kanal,adresse,bankA,iOutA)
        plexer.writeByteData(kanal,adresse,bankB,iOutB)
        #Prüfen und antworten.
        iOutA=plexer.readByteData(kanal,adresse,bankA)
        iOutB=plexer.readByteData(kanal,adresse,bankB)
        publish_output_bits(kanal, adresse, (iOutB << 8) | iOutA)
        sStatus="OK"      
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler Output lesen"
        log("Fehler Output: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        publish_command_event("SOM", channel=kanal, address=hex(adresse), value=value, status=sStatus.replace(";",""))  
        
def log(message: str, level: str = "INFO") -> None:
    """
    Logging-Funktion für GeCoS-Server
    
    Args:
        message: Nachricht zum Loggen
        level: Log-Level (INFO, WARNING, ERROR, DEBUG)
    """
    if level == "ERROR":
        logger.error(message)
    elif level == "WARNING":
        logger.warning(message)
    elif level == "DEBUG":
        logger.debug(message)
    else:
        logger.info(message)


def mqtt_publish(topic: str, payload: Any, retain: bool = False) -> None:
    """Veröffentlicht eine Nachricht auf dem MQTT-Bus."""
    if not mqtt_client:
        log(f"MQTT Client nicht initialisiert (Topic: {topic})", "ERROR")
        return
    if isinstance(payload, bytes):
        message = payload.decode("utf-8", errors="ignore")
    elif isinstance(payload, str):
        message = payload
    else:
        try:
            message = json.dumps(payload, ensure_ascii=True)
        except (TypeError, ValueError) as exc:
            log(f"MQTT JSON Fehler {topic}: {exc}", "ERROR")
            return
    try:
        mqtt_client.publish(topic, message, retain=retain)
    except Exception as exc:
        log(f"MQTT Publish Fehler {topic}: {exc}", "ERROR")


def publish_state_event(event: str, payload: Any, retain: bool = False) -> None:
    """Hilfsfunktion zum Veröffentlichen von Zuständen im State-Topic."""
    state_base = mqtt_topics.get("state")
    if not state_base:
        return
    event_name = event.lower() if event else "raw"
    mqtt_publish(f"{state_base}/{event_name}", payload, retain=retain)


def publish_command_event(event: str, **fields: Any) -> None:
    payload = {"event": event}
    payload.update({k: v for k, v in fields.items() if v is not None})
    publish_state_event(event, payload)


def publish_input_bits(kanal: int, adresse: int, value: int) -> None:
    topic_base = mqtt_topics.get("inputs")
    if topic_base is None:
        return
    old_value = input_state_cache.get((kanal, adresse))
    if old_value == value:
        return
    for bit in range(16):
        mask = 1 << bit
        new_state = bool(value & mask)
        if old_value is None or new_state != bool(old_value & mask):
            topic = f"{topic_base}/{kanal}/{adresse:02x}/{bit}"
            mqtt_publish(topic, "ON" if new_state else "OFF", retain=True)
    input_state_cache[(kanal, adresse)] = value


def publish_output_bits(kanal: int, adresse: int, value: int) -> None:
    topic_base = mqtt_topics.get("outputs")
    if topic_base is None:
        return
    old_value = output_state_cache.get((kanal, adresse))
    if old_value == value:
        return
    for bit in range(16):
        mask = 1 << bit
        new_state = bool(value & mask)
        if old_value is None or new_state != bool(old_value & mask):
            topic = f"{topic_base}/{kanal}/{adresse:02x}/{bit}"
            mqtt_publish(topic, "ON" if new_state else "OFF", retain=True)
    output_state_cache[(kanal, adresse)] = value


def publish_pwm_channel(kanal: int, adresse: int, channel: int, enabled: bool, value: int) -> None:
    topic_base = mqtt_topics.get("pwm")
    if topic_base is None:
        return
    key = (kanal, adresse, channel)
    clamped_value = max(0, min(4095, int(value)))
    effective_value = clamped_value if enabled else 0
    new_state = (1 if enabled else 0, clamped_value)
    if pwm_state_cache.get(key) == new_state:
        return
    percent = 0
    if effective_value > 0:
        percent = round(effective_value / 4095 * 100)
    topic = f"{topic_base}/{kanal}/{adresse:02x}/{channel}"
    payload = {
        "state": "ON" if enabled and effective_value > 0 else "OFF",
        "brightness": percent
    }
    mqtt_publish(topic, payload, retain=True)
    pwm_state_cache[key] = new_state


def publish_analog_value(kanal: int, adresse: int, channel: int, value: float) -> None:
    topic_base = mqtt_topics.get("analog")
    if topic_base is None:
        return
    key = (kanal, adresse, channel)
    rounded = round(float(value), 4)
    if analog_state_cache.get(key) == rounded:
        return
    topic = f"{topic_base}/{kanal}/{adresse:02x}/{channel}"
    mqtt_publish(topic, f"{rounded}", retain=True)
    analog_state_cache[key] = rounded


def _ow_latch_is_on(state_byte: int, pin: str) -> bool:
    """DS2413: Latch-Bit 0 bedeutet Ausgang aktiv (Open-Drain schaltet nach Masse)"""
    mask = 0x02 if pin == "a" else 0x08
    return not bool(int(state_byte) & mask)


def publish_ow_availability(address: str, available: bool) -> None:
    """Meldet pro OneWire-Gerät, ob der letzte Lesevorgang erfolgreich war."""
    topic_base = mqtt_topics.get("onewire")
    if topic_base is None:
        return
    if ow_avail_cache.get(address) == available:
        return
    mqtt_publish(f"{topic_base}/{address}/status", "online" if available else "offline", retain=True)
    ow_avail_cache[address] = available


def publish_ow_state(address: str, value: Any, status: str) -> None:
    """Veröffentlicht Messwert bzw. Schaltzustand eines OneWire-Geräts."""
    topic_base = mqtt_topics.get("onewire")
    if topic_base is None:
        return
    family = address.split("-")[0].lower()
    try:
        available = status == "OK" and float(value) != -85
    except (TypeError, ValueError):
        available = False
    publish_ow_availability(address, available)
    if not available:
        return
    if family in OW_FAMILY_TEMPERATURE:
        rounded = round(float(value), 2)
        if ow_state_cache.get(address) == rounded:
            return
        mqtt_publish(f"{topic_base}/{address}/temperature", f"{rounded}", retain=True)
        ow_state_cache[address] = rounded
    elif family == OW_FAMILY_SWITCH:
        state_byte = int(value)
        if ow_state_cache.get(address) == state_byte:
            return
        for pin in ("a", "b"):
            payload = "ON" if _ow_latch_is_on(state_byte, pin) else "OFF"
            mqtt_publish(f"{topic_base}/{address}/{pin}", payload, retain=True)
        ow_state_cache[address] = state_byte


def notify_invalid_command(arr: List[str], reason: str) -> None:
    payload = ";".join(arr)
    publish_command_event("ERR", payload=payload, reason=reason)
    log(f"{reason}: {payload}", "ERROR")


def _percent_to_pwm_value(percent: float) -> int:
    """Wandelt einen Prozentwert (0-100) in einen 12-Bit PWM Wert um."""
    try:
        pct = float(percent)
    except (TypeError, ValueError):
        return 0
    pct = max(0.0, min(100.0, pct))
    return int(round(pct / 100 * 4095))


def dispatch_command(arr: List[str]) -> None:
    if not arr:
        return
    cmd = arr[0].upper()
    if cmd == "MOD":
        modulSuche()
    elif cmd == "OWS":
        thread_OW_Search()
    elif cmd == "SAI":
        interrutpKanal(intKanal0)
        interrutpKanal(intKanal1)
        interrutpKanal(intKanal2)
    elif cmd == "SPWM":
        pwmAll()
    elif cmd == "SRGBW":
        rgbwAll()
    elif cmd == "SAO":
        ReadOutAll()
    elif cmd == "RRTC":
        read_rtc()
    elif cmd == "SOM":
        if len(arr) >= 4:
            set_output(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "PWM":
        if len(arr) >= 6:
            set_pwm(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "RGBW":
        if len(arr) >= 10:
            set_rgbw(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "SAM":
        if len(arr) >= 6:
            read_analog(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "OWV":
        if len(arr) >= 2:
            thread_OW_read(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "OWC":
        if len(arr) >= 3:
            thread_OW_config(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    elif cmd == "SRTC":
        if len(arr) >= 7:
            set_rtc(arr)
        else:
            notify_invalid_command(arr, "Parameter fehlen")
    else:
        notify_invalid_command(arr, "Befehl nicht erkannt")


def handle_raw_command(payload: str) -> None:
    data = (payload or "").strip()
    if not data:
        return
    if data.startswith("{") and data.endswith("}"):
        data = data[1:-1]
    if not data:
        return
    if ";" not in data:
        dispatch_command([data])
        return
    parts = data.split(";")
    dispatch_command(parts)


def handle_output_command(channel_str: str, address_str: str, bit_str: str, payload: str) -> None:
    try:
        kanal = int(channel_str)
        adresse = int(address_str, 16)
        bit = int(bit_str)
    except ValueError:
        log(f"Ungültige Output-Parameter: {channel_str}/{address_str}/{bit_str}", "ERROR")
        return
    if bit < 0 or bit > 15:
        log(f"Ungültiger Output-Bit: {bit}", "ERROR")
        return
    normalized = (payload or "").strip().upper()
    if normalized in ("1", "ON", "TRUE"):
        state = True
    elif normalized in ("0", "OFF", "FALSE"):
        state = False
    else:
        log(f"Ungültiger Output-Payload: {payload}", "ERROR")
        return
    current_value = output_state_cache.get((kanal, adresse))
    if current_value is None:
        read_output(kanal, adresse)
        current_value = output_state_cache.get((kanal, adresse), 0)
    new_value = set_bit(current_value, bit, state)
    arr = ["SOM", str(kanal), hex(adresse), str(new_value)]
    set_output(arr)


def handle_pwm_command(channel_str: str, address_str: str, pwm_channel_str: str, payload: str) -> None:
    try:
        kanal = int(channel_str)
        adresse = int(address_str, 16)
        pwm_channel = int(pwm_channel_str)
    except ValueError:
        log(f"Ungültige PWM-Parameter: {channel_str}/{address_str}/{pwm_channel_str}", "ERROR")
        return
    if pwm_channel < 0 or pwm_channel > 15:
        log(f"Ungültiger PWM-Kanal: {pwm_channel}", "ERROR")
        return
    normalized = (payload or "").strip()
    if not normalized:
        log("Leerer PWM-Payload", "ERROR")
        return
    status: Optional[int] = None
    value: Optional[int] = None
    if normalized.startswith("{") and normalized.endswith("}"):
        try:
            data = json.loads(normalized)
        except (TypeError, ValueError) as exc:
            log(f"Ungültiges PWM JSON: {exc}", "ERROR")
            return
        if "value" in data:
            try:
                value = int(float(data["value"]))
            except (TypeError, ValueError):
                log("PWM value ungültig", "ERROR")
                return
        elif "brightness" in data:
            try:
                brightness = float(data["brightness"])
                value = _percent_to_pwm_value(brightness)
            except (TypeError, ValueError):
                log("PWM brightness ungültig", "ERROR")
                return
        if "state" in data:
            state = str(data["state"]).strip().upper()
            if state in ("ON", "1", "TRUE"):
                status = 1
            elif state in ("OFF", "0", "FALSE"):
                status = 0
    else:
        upper = normalized.upper()
        if upper in ("ON", "1", "TRUE"):
            status = 1
        elif upper in ("OFF", "0", "FALSE"):
            status = 0
        else:
            try:
                numeric = float(normalized)
                if 0 <= numeric <= 100:
                    value = _percent_to_pwm_value(numeric)
                else:
                    value = int(numeric)
            except ValueError:
                log(f"Ungültiger PWM-Payload: {payload}", "ERROR")
                return
    cached = pwm_state_cache.get((kanal, adresse, pwm_channel))
    if value is None:
        if status == 1 and cached:
            value = cached[1] if cached[1] > 0 else 4095
        elif status == 0:
            value = 0
        else:
            value = 4095
    value = max(0, min(4095, value))
    if status is None:
        status = 1 if value > 0 else 0
    if status == 0:
        effective_value = 0
    else:
        effective_value = value
    arr = [
        "PWM",
        str(kanal),
        hex(adresse),
        str(pwm_channel),
        str(status),
        str(effective_value)
    ]
    set_pwm(arr)


def handle_ow_command(address: str, pin: str, payload: str) -> None:
    pin = pin.lower()
    if pin not in ("a", "b"):
        log(f"Ungültiger OneWire-Pin: {pin}", "ERROR")
        return
    if address.split("-")[0].lower() != OW_FAMILY_SWITCH:
        log(f"OneWire-Gerät {address} ist kein schaltbarer Ausgang", "ERROR")
        return
    normalized = (payload or "").strip().upper()
    if normalized in ("1", "ON", "TRUE"):
        state = True
    elif normalized in ("0", "OFF", "FALSE"):
        state = False
    else:
        log(f"Ungültiger OneWire-Payload: {payload}", "ERROR")
        return
    thread_OW_switch(address, pin, state)


def on_mqtt_connect(client: mqtt.Client, _userdata, _flags, rc: int) -> None:
    if rc == 0:
        mqtt_connected.set()
        command_topic = mqtt_topics.get("command")
        availability = mqtt_topics.get("availability")
        if command_topic:
            client.subscribe(f"{command_topic}/#")
        if availability:
            mqtt_publish(availability, "online", retain=True)
        log("MQTT Verbindung hergestellt", "INFO")
    else:
        log(f"MQTT Verbindungsfehler (rc={rc})", "ERROR")


def on_mqtt_disconnect(_client: mqtt.Client, _userdata, rc: int) -> None:
    mqtt_connected.clear()
    log(f"MQTT Verbindung getrennt (rc={rc})", "WARNING")


def on_mqtt_message(_client: mqtt.Client, _userdata, msg) -> None:
    try:
        payload = msg.payload.decode("utf-8", errors="ignore")
    except Exception:
        payload = ""
    command_prefix = mqtt_topics.get("command")
    if not command_prefix:
        return
    prefix = f"{command_prefix}/"
    if msg.topic.startswith(prefix):
        command_path = msg.topic[len(prefix):]
        if command_path.startswith("raw"):
            handle_raw_command(payload)
        elif command_path.startswith("output/"):
            parts = command_path.split("/")
            if len(parts) == 4:
                handle_output_command(parts[1], parts[2], parts[3], payload)
            else:
                log(f"Ungültiges Output-Topic: {msg.topic}", "ERROR")
        elif command_path.startswith("pwm/"):
            parts = command_path.split("/")
            if len(parts) == 4:
                handle_pwm_command(parts[1], parts[2], parts[3], payload)
            else:
                log(f"Ungültiges PWM-Topic: {msg.topic}", "ERROR")
        elif command_path.startswith("onewire/"):
            parts = command_path.split("/")
            if len(parts) == 3:
                handle_ow_command(parts[1], parts[2], payload)
            else:
                log(f"Ungültiges OneWire-Topic: {msg.topic}", "ERROR")
        else:
            log(f"Unbekanntes MQTT Topic: {msg.topic}", "DEBUG")


def init_mqtt(args) -> None:
    global mqtt_client, mqtt_topics, mqtt_settings
    base_topic = (args.mqtt_base_topic or "gecos").strip().strip("/")
    if not base_topic:
        base_topic = "gecos"
    ha_prefix = (args.ha_prefix or "homeassistant").strip().strip("/")
    device_name = (args.device_name or "GeCoS Server").strip()
    mqtt_settings = {
        "host": args.mqtt_host,
        "port": args.mqtt_port,
        "username": args.mqtt_username,
        "password": args.mqtt_password,
        "client_id": args.mqtt_client_id or f"gecos-server-{socket.gethostname()}",
        "base_topic": base_topic,
        "ha_discovery": args.ha_discovery,
        "ha_prefix": ha_prefix,
        "device_name": device_name,
        "keepalive": args.mqtt_keepalive,
        "ow_interval": max(0, args.ow_interval),
        "ow_active_pullup": not args.ow_no_active_pullup
    }
    mqtt_topics = {
        "base": base_topic,
        "state": f"{base_topic}/state",
        "command": f"{base_topic}/command",
        "availability": f"{base_topic}/status",
        "inputs": f"{base_topic}/inputs",
        "outputs": f"{base_topic}/outputs",
        "pwm": f"{base_topic}/pwm",
        "analog": f"{base_topic}/analog",
        "onewire": f"{base_topic}/onewire"
    }
    #paho-mqtt 2.x erwartet die Callback-Version als erstes Argument, 1.x kennt den Parameter nicht
    callback_api = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api is not None:
        mqtt_client = mqtt.Client(callback_api.VERSION1, client_id=mqtt_settings["client_id"])
    else:
        mqtt_client = mqtt.Client(client_id=mqtt_settings["client_id"])
    if mqtt_settings["username"]:
        mqtt_client.username_pw_set(mqtt_settings["username"], mqtt_settings["password"])
    mqtt_client.on_connect = on_mqtt_connect
    mqtt_client.on_disconnect = on_mqtt_disconnect
    mqtt_client.on_message = on_mqtt_message
    availability_topic = mqtt_topics["availability"]
    mqtt_client.will_set(availability_topic, payload="offline", retain=True)
    try:
        mqtt_client.connect(mqtt_settings["host"], mqtt_settings["port"], keepalive=mqtt_settings["keepalive"])
    except Exception as exc:
        log(f"MQTT Verbindung zu {mqtt_settings['host']}:{mqtt_settings['port']} fehlgeschlagen: {exc}", "ERROR")
        raise
    mqtt_client.loop_start()
    if not mqtt_connected.wait(timeout=10):
        log("MQTT Verbindung Timeout", "ERROR")


def publish_ha_discovery() -> None:
    if not mqtt_settings.get("ha_discovery"):
        return
    ha_prefix = mqtt_settings.get("ha_prefix")
    state_inputs = mqtt_topics.get("inputs")
    state_outputs = mqtt_topics.get("outputs")
    state_pwm = mqtt_topics.get("pwm")
    state_analog = mqtt_topics.get("analog")
    state_ow = mqtt_topics.get("onewire")
    availability = mqtt_topics.get("availability")
    command_topic = mqtt_topics.get("command")
    if not all([ha_prefix, state_inputs, state_outputs, availability, command_topic]):
        return
    device_id = mqtt_settings.get("client_id", "gecos-server")
    device_info = {
        "identifiers": [device_id],
        "name": mqtt_settings.get("device_name", "GeCoS Server"),
        "manufacturer": "GeCoS",
        "model": "I2C Controller",
        "sw_version": "MQTT"
    }
    desired_topics: Set[str] = set()
    # Inputs als Binary Sensoren anmelden
    for kanal, addresses in modules['in'].items():
        for adresse in addresses:
            for bit in range(16):
                object_id = f"{device_id}_in_{kanal}_{adresse:02x}_{bit}"
                config_topic = f"{ha_prefix}/binary_sensor/{object_id}/config"
                desired_topics.add(config_topic)
                payload = {
                    "name": f"GeCoS IN {kanal}-{adresse:02x} #{bit:02d}",
                    "state_topic": f"{state_inputs}/{kanal}/{adresse:02x}/{bit}",
                    "payload_on": "ON",
                    "payload_off": "OFF",
                    "availability_topic": availability,
                    "unique_id": object_id,
                    "device": device_info
                }
                mqtt_publish(config_topic, payload, retain=True)
    # Outputs als Switches anmelden
    for kanal, addresses in modules['out'].items():
        for adresse in addresses:
            for bit in range(16):
                object_id = f"{device_id}_out_{kanal}_{adresse:02x}_{bit}"
                config_topic = f"{ha_prefix}/switch/{object_id}/config"
                desired_topics.add(config_topic)
                payload = {
                    "name": f"GeCoS OUT {kanal}-{adresse:02x} #{bit:02d}",
                    "state_topic": f"{state_outputs}/{kanal}/{adresse:02x}/{bit}",
                    "command_topic": f"{command_topic}/output/{kanal}/{adresse:02x}/{bit}",
                    "payload_on": "ON",
                    "payload_off": "OFF",
                    "availability_topic": availability,
                    "unique_id": object_id,
                    "device": device_info
                }
                mqtt_publish(config_topic, payload, retain=True)
    # PWM als Light-Entitäten (Dimmer) anmelden
    if state_pwm:
        for kanal, addresses in modules['pwm'].items():
            for adresse in addresses:
                for channel in range(16):
                    object_id = f"{device_id}_pwm_{kanal}_{adresse:02x}_{channel}"
                    config_topic = f"{ha_prefix}/light/{object_id}/config"
                    desired_topics.add(config_topic)
                    payload = {
                        "name": f"GeCoS PWM {kanal}-{adresse:02x} #{channel:02d}",
                        "schema": "json",
                        "state_topic": f"{state_pwm}/{kanal}/{adresse:02x}/{channel}",
                        "command_topic": f"{command_topic}/pwm/{kanal}/{adresse:02x}/{channel}",
                        "brightness": True,
                        "brightness_scale": 100,
                        "availability_topic": availability,
                        "unique_id": object_id,
                        "device": device_info,
                        "icon": "mdi:led-strip-variant"
                    }
                    mqtt_publish(config_topic, payload, retain=True)
    # Analogeingänge als Sensoren anmelden
    if state_analog:
        for kanal, addresses in modules['ana'].items():
            for adresse in addresses:
                for channel in range(4):
                    object_id = f"{device_id}_ana_{kanal}_{adresse:02x}_{channel}"
                    config_topic = f"{ha_prefix}/sensor/{object_id}/config"
                    desired_topics.add(config_topic)
                    payload = {
                        "name": f"GeCoS ANA {kanal}-{adresse:02x} CH{channel}",
                        "state_topic": f"{state_analog}/{kanal}/{adresse:02x}/{channel}",
                        "availability_topic": availability,
                        "unique_id": object_id,
                        "device": device_info,
                        "device_class": "voltage",
                        "state_class": "measurement",
                        "unit_of_measurement": "V"
                    }
                    mqtt_publish(config_topic, payload, retain=True)
    # OneWire-Geraete anmelden (Temperatur als Sensor, DS2413 als zwei Switches)
    if state_ow:
        for address in modules['ow']:
            family = address.split("-")[0].lower()
            slug = address.replace("-", "_")
            availability_ow = [
                {"topic": availability},
                {"topic": f"{state_ow}/{address}/status"}
            ]
            if family in OW_FAMILY_TEMPERATURE:
                object_id = f"{device_id}_ow_{slug}"
                config_topic = f"{ha_prefix}/sensor/{object_id}/config"
                desired_topics.add(config_topic)
                payload = {
                    "name": f"GeCoS 1Wire {address}",
                    "state_topic": f"{state_ow}/{address}/temperature",
                    "availability": availability_ow,
                    "availability_mode": "all",
                    "unique_id": object_id,
                    "device": device_info,
                    "device_class": "temperature",
                    "state_class": "measurement",
                    "unit_of_measurement": "°C"
                }
                mqtt_publish(config_topic, payload, retain=True)
            elif family == OW_FAMILY_SWITCH:
                for pin in ("a", "b"):
                    object_id = f"{device_id}_ow_{slug}_{pin}"
                    config_topic = f"{ha_prefix}/switch/{object_id}/config"
                    desired_topics.add(config_topic)
                    payload = {
                        "name": f"GeCoS 1Wire {address} PIO {pin.upper()}",
                        "state_topic": f"{state_ow}/{address}/{pin}",
                        "command_topic": f"{command_topic}/onewire/{address}/{pin}",
                        "payload_on": "ON",
                        "payload_off": "OFF",
                        "availability": availability_ow,
                        "availability_mode": "all",
                        "unique_id": object_id,
                        "device": device_info,
                        "icon": "mdi:electric-switch"
                    }
                    mqtt_publish(config_topic, payload, retain=True)
            else:
                log(f"OneWire {address}: Family-Code ohne HA-Entity", "DEBUG")
    stale = ha_discovery_topics - desired_topics
    for topic in stale:
        mqtt_publish(topic, "", retain=True)
    ha_discovery_topics.clear()
    ha_discovery_topics.update(desired_topics)

def set_bit(v: int, index: int, x: bool) -> int:
    """Setzt ein einzelnes Bit auf 1/0 (True oder False)"""
    #Bit auf 1/0 setzen (True oder False)
    mask = 1<< index
    v&=~mask
    if x:
        v |= mask
    return v
    
def ReadOutAll():
    """Liest alle Output-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['out'][kanal]:
            try:
                read_output(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von Output Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")
        

def pwmAll():
    """Liest alle PWM-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['pwm'][kanal]:
            try:
                read_pwm(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von PWM Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")


def rgbwAll():
    """Liest alle RGBW-Module auf allen Kanälen"""
    for kanal in range(3):
        for device in modules['rgbw'][kanal]:
            try:
                read_rgbw(kanal, device)
            except Exception as e:
                log(f"Fehler beim Lesen von RGBW Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")    

def interrutpKanal(pin):
    """Verarbeitet Interrupt für einen bestimmten Pin/Kanal"""
    # Kanal nach INT Pin Wählen:
    if pin==intKanal0:
        kanal=0
    elif pin==intKanal1:
        kanal=1
    elif pin==intKanal2:
        kanal=2
    else:
        log("Kanal ungültig","ERROR")
        return
    
    for device in modules['in'][kanal]:
        try:
            read_input(kanal, device, 1)
        except Exception as e:
            log(f"Fehler beim Lesen von Input Kanal {kanal}, Device {hex(device)}: {e}", "ERROR")

def read_rtc():
    try:
        rtctime = ds.read_datetime()
        publish_command_event(
            "RRTC",
            timestamp=rtctime.strftime("%d.%m.%Y %H:%M:%S"),
            temperature=ds.read_temp(),
            status="OK"
        )
    except Exception as e:
        publish_command_event("RRTC", status="Fehler RTC lesen", message=str(e))
        log("Error RTC lesen:" + str(e),"ERROR") 


def set_rtc(arr):
    try:
        str_dto= "{0}/{1}/{2} {3}:{4}:{5}".format(arr[2],arr[1],arr[3],arr[4],arr[5],arr[6])
        dto = datetime.strptime(str(str_dto), '%m/%d/%Y %H:%M:%S')
        ds.write_datetime(dto)
        publish_command_event("SRTC", payload=";".join(arr), status="OK")
    except (ValueError, IndexError) as e: 
        publish_command_event("SRTC", payload=";".join(arr), status="Fehler RTC setzen")
        log(f"Error RTC setzen: {e}","ERROR") 
        

def read_analog(arr):
    # "SAM";I2C Kanal;Adresse;Channel-Analog;Resolution;Amplifier
    # {SAM;0;0x69;AnalogChannel;Resolution;Amplifier}
    # {SAM;0;0x69;0;3;0}
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    channel=int(arr[3])
    res=int(arr[4])
    amp=int(arr[5])
    if adresse <0x68 or adresse > 0x6B:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Modul adresse ungueltig")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Kanal ungueltig")
        return
        
    if channel <0 or channel > 3:
        log("Analog Channel ungueltig","ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Analog Channel ungueltig")
        return
    if res <0 or res > 3:
        log("Analog Resolution ungueltig","ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Analog Resolution ungueltig")
        return
    if amp <0 or amp > 3:
        log("Analog Amplifier ungueltig","ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Analog Amplifier ungueltig")
        return
            
    #Config Bits bit5+6 = Channel
    # Bit 4  4Converison Mode = 1
    # Bits 3+2 Resolution
    # Bist 0+1 = Amplifier
    #arr[3] = Resolution  
    #arr[4] = Amplifier
    bconfig=b"0"
    bconfig = channel <<5 | 1 <<4 | res <<2 | amp
    plexer.writeByte(kanal,adresse,bconfig)
    #Warten bis ergebnis:
    #I2C Port Freigeben:
    if res==0:
        time.sleep(0.010)
    elif res==1:
        time.sleep(0.022)
    elif res==2:
        time.sleep(0.080)
    else:
        time.sleep(0.300)
    #Je Nach Auflösung 3 oder 4Byte lesen:
    #res=3 dann 4 sonst 3
    readyBit=0
    if res==3:
        erg=plexer.readBlockData(kanal,adresse,bconfig,4)
        readyBit=bit_from_string(erg[3],8)
    else:
        erg=plexer.readBlockData(kanal,adresse,bconfig,3)
        readyBit=bit_from_string(erg[2],8)

    signBit=0
    if readyBit==0:
        if res==0:
            #12bit
            wert = ((erg[0] & 0b00001111) <<8 | erg[1])
            signBit=bit_from_string(wert,11)
            if signBit:
                wert = set_bit(wert,11,0)
            wert=wert*0.004923
            if signBit:
                wert=wert-2048               

        elif res==1:
            #14bit
            wert = ((erg[0] & 0b00111111) <<8 | erg[1])
            signBit=bit_from_string(wert,13)
            if signBit:
                wert = set_bit(wert,13,0)
            wert=wert*0.00123075
            if signBit:
                wert=wert-2048

        elif res==2:
            #16bit
            wert = (erg[0] <<8 | erg[1])
            signBit=bit_from_string(wert,15)
            if signBit:
                wert = set_bit(wert,15,0)
            wert=wert*0.0003076875
            if signBit:
                wert=wert-2048
        else:
            #18bit
            wert = ((erg[0] & 0b00000011) <<16 | erg[1]<<8 | erg[2])
            signBit=bit_from_string(wert,17)
            if signBit:
                wert = set_bit(wert,17,0)
            wert=wert*0.000076921875
            if signBit:
                wert=wert-2048
        sStatus="OK"
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        measured = round(wert,3)
        publish_analog_value(kanal, adresse, channel, measured)
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, value=measured, status=sStatus)
    else:
        log("Analog: Daten nicht bereit...","ERROR")
        publish_command_event("SAM", channel=kanal, address=hex(adresse), analog_channel=channel, status="Analog Daten nicht bereit")
        return

def read_pwm(kanal, adresse):
    if adresse <0x50 or adresse > 0x57:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("SPWM", channel=kanal, address=hex(adresse), status="Modul adresse ungueltig")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        publish_command_event("SPWM", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return
    
    #{PWM;I2C-Kanal;Adresse;Kanal;Wert}
    #befehl="{0};{1};".format(kanal,hex(adresse))
    for i in range(16): #16
        startAdr=int(i*4+6)
        #LowByte
        lByte=plexer.readByteData(kanal,adresse,startAdr+2)
        #HighByte
        hByte=plexer.readByteData(kanal,adresse,startAdr+3)
        tmpByte=0
        tmpByte=(hByte >> 4) & 0b0000001
        wert=0
        wert = wert*256+int(hByte& 0b0001111)
        wert = wert*256+int(lByte)
        if wert==0:
            wert=0
        else:
            wert=wert
        is_enabled = (tmpByte == 0)
        publish_pwm_channel(kanal, adresse, i, is_enabled, wert)
        publish_command_event(
            "SPWM",
            channel=kanal,
            address=hex(adresse),
            pwm_channel=i,
            enabled=is_enabled,
            value=wert,
            status="OK"
        )

def read_rgbw(kanal, adresse):
    if adresse <0x57 or adresse > 0x5f:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("SRGBW", channel=kanal, address=hex(adresse), status="Modul adresse ungueltig")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        publish_command_event("SRGBW", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return
        
    #{RGBW;I2C-Kanal;Adresse;RGBWKanal;StatusRGB;StatusW;R;G;B;W}
    #befehl="{0};{1};".format(kanal,hex(adresse))
    i2 = 0
    hByteW=0
    hByteR=0
    r=0
    g=0
    b=0
    w=0
    i3=0
    for i in range(16): #16
        startAdr=int(i*4+6)
        #LowByte
        lByte=plexer.readByteData(kanal,adresse,startAdr+2)
        #HighByte
        hByte=plexer.readByteData(kanal,adresse,startAdr+3)
        wert=0
        wert = wert*256+int(hByte& 0b0001111)
        wert = wert*256+int(lByte)
        if i3==0:
            r=wert
            hByteR=hByte
        elif i3==1:
            g=wert
        elif i3==2:
            b=wert
        elif i3==3:
            w=wert
            hByteW=hByte

        if i2==0:
              if i == i2+3:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==0:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==0:
                    iSR=1
                else:
                    iSR=0
                publish_command_event(
                    "SRGBW",
                    channel=kanal,
                    address=hex(adresse),
                    group=i2-1,
                    status_rgb=iSR,
                    status_w=iSW,
                    r=r,
                    g=g,
                    b=b,
                    w=w,
                    status="OK"
                )
                r=0
                g=0
                b=0
                w=0
        if i2==1:
            if i == i2+6:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==1:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==1:
                    iSR=1
                else:
                    iSR=0
                publish_command_event(
                    "SRGBW",
                    channel=kanal,
                    address=hex(adresse),
                    group=i2-1,
                    status_rgb=iSR,
                    status_w=iSW,
                    r=r,
                    g=g,
                    b=b,
                    w=w,
                    status="OK"
                )
                r=0
                g=0
                b=0
                w=0
        if i2==2:
            if i== i2+9:
                i2+=1
                #PWM Status W
                iSW=0
                tmpByte=(hByteW >> 4) & 0b0000001
                if tmpByte==1:
                    iSW=1
                else:
                    iSW=0
                 #PWM Status R
                iSR=0
                tmpByte=(hByteR >> 4) & 0b0000001
                if tmpByte==1:
                    iSR=1
                else:
                    iSR=0
                publish_command_event(
                    "SRGBW",
                    channel=kanal,
                    address=hex(adresse),
                    group=i2-1,
                    status_rgb=iSR,
                    status_w=iSW,
                    r=r,
                    g=g,
                    b=b,
                    w=w,
                    status="OK"
                )
                r=0
                g=0
                b=0
                w=0
        if i==15:
            i2+=1
            #PWM Status W
            iSW=0
            tmpByte=(hByteW >> 4) & 0b0000001
            if tmpByte==1:
                iSW=1
            else:
                iSW=0
            #PWM Status R
            iSR=0
            tmpByte=(hByteR >> 4) & 0b0000001
            if tmpByte==1:
                iSR=1
            else:
                iSR=0
            publish_command_event(
                "SRGBW",
                channel=kanal,
                address=hex(adresse),
                group=i2-1,
                status_rgb=iSR,
                status_w=iSW,
                r=r,
                g=g,
                b=b,
                w=w,
                status="OK"
            )
            r=0
            g=0
            b=0
            w=0

def set_pwm(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    pwm_value = int(arr[5]) if len(arr) > 5 else 0
    if adresse <0x50 or adresse > 0x57:
        log("Modul Adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("PWM", channel=kanal, address=hex(adresse), status="Modul Adresse ungueltig")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        publish_command_event("PWM", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return
    pwm_channel = int(arr[3])
    if pwm_channel < 0 or pwm_channel >15:
        log("PWM-Kanal ungueltig","ERROR")
        publish_command_event("PWM", channel=kanal, address=hex(adresse), pwm_channel=pwm_channel, status="PWM-Kanal ungueltig")
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("PWM-Status ungueltig","ERROR")
        publish_command_event("PWM", channel=kanal, address=hex(adresse), pwm_channel=pwm_channel, status="PWM-Status ungueltig")
        return
    if pwm_value <0 or pwm_value >4095:
        log("PWM-Wert ungueltig","ERROR")
        publish_command_event("PWM", channel=kanal, address=hex(adresse), pwm_channel=pwm_channel, status="PWM-Wert ungueltig")
        return
    sStatus=""
    try:
        #LED_ON Immer 0
        #LED_OFF 4096*X%-1
        #Array durchlaufen 0-15 (+1) = ausgang; ausgang*4+6 = Start Adresse LED_ON_L 
        #Array 3= Kanal 4 = wert
        wert = pwm_value
        startAdr=int(pwm_channel*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        publish_pwm_channel(kanal, adresse, pwm_channel, int(arr[4]) == 1, wert)
        sStatus="OK"
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(err),"ERROR")
    except:
        sStatus="Fehler PWM Setzen lesen"
        log("Fehler PWM Setzen: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        publish_command_event(
            "PWM",
            channel=kanal,
            address=hex(adresse),
            pwm_channel=pwm_channel,
            value=pwm_value,
            enabled=int(arr[4]) == 1,
            status=sStatus.replace(";","")
        )

def set_rgbw(arr):
    adresse=int(arr[2],16)
    kanal=int(arr[1])
    if adresse <0x58 or adresse > 0x5f:
        log("Modul Adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("RGBW", channel=kanal, address=hex(adresse), status="Modul Adresse ungueltig")
        return
    
    if kanal <0 or kanal > 3:
        log("Kanal ungueltig","ERROR")
        publish_command_event("RGBW", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return
    if int(arr[3]) <0 or int(arr[3]) >3:
        log("PWMKanal ungueltig","ERROR")
        publish_command_event("RGBW", channel=kanal, address=hex(adresse), status="PWM-Kanal ungueltig")
        return
    if int(arr[4]) <0 or int(arr[4]) >1:
        log("StatusRGB ungueltig","ERROR")
        publish_command_event("RGBW", channel=kanal, address=hex(adresse), status="StatusRGB ungueltig")
        return
    if int(arr[5]) <0 or int(arr[5]) >1:
        log("StatusW ungueltig","ERROR")
        publish_command_event("RGBW", channel=kanal, address=hex(adresse), status="StatusW ungueltig")
        return
    sStatus=""
    try:
        #LED_ON Immer 0
        #LED_OFF 4096*X%-1
        #Array durchlaufen 0-15 (+1) = ausgang; ausgang*4+6 = Start Adresse LED_ON_L 
        #Array 3= Kanal 4 = wert
        i=int(arr[3])
        if i==1:
            i+=3
        elif i==2:
            i+=6
        elif i==3:
            i+=9
        r=int(arr[6])
        g=int(arr[7])
        b=int(arr[8])
        w=int(arr[9])
        #Rot:
        wert = r #int(round(4095*(r/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Grün:
        wert = g #int(round(4095*(g/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Blau:
        wert = b #int(round(4095*(b/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[4])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        i+=1
        #Weiß:
        wert = w #int(round(4095*(w/100)))
        startAdr=int(i*4+6)
        hByte, lByte = bytes(divmod(wert,0x100))
        #Status Ein/Aus:
        if(int(arr[5])==1):
            hByte=set_bit(hByte,4,False)
        else:
            hByte=set_bit(hByte,4,True)
        plexer.writeByteData(kanal,adresse,startAdr,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+1,0x00)
        plexer.writeByteData(kanal,adresse,startAdr+2,lByte)
        plexer.writeByteData(kanal,adresse,startAdr+3,hByte)
        sStatus="OK"
    except OSError as err:
        sStatus=str(err)
        log("I/O error: {0}".format(str(err)),"ERROR")
    except:
        sStatus="Fehler PWM Setzen lesen"
        log("Fehler PWM Setzen: {0}".format(arr),"ERROR")
    finally:
        if len(sStatus) < 1:
            sStatus="Unkown Error"
        publish_command_event(
            "RGBW",
            channel=kanal,
            address=hex(adresse),
            group=int(arr[3]),
            status_rgb=int(arr[4]),
            status_w=int(arr[5]),
            r=int(arr[6]),
            g=int(arr[7]),
            b=int(arr[8]),
            w=int(arr[9]),
            status=sStatus.replace(";","")
        )   
        
def read_input(kanal,adresse, manual=0):
    """Liest Eingänge von einem Modul und sendet bei Änderung"""
    if adresse <0x20 or adresse > 0x23:
        log("Modul adresse ungueltig: {0}".format(adresse),"ERROR")
        publish_command_event("SAI", channel=kanal, address=hex(adresse), status="Modul adresse ungueltig")
        return
    
    if kanal <0 or kanal > 2:
        log("Kanal ungueltig","ERROR")
        publish_command_event("SAI", channel=kanal, address=hex(adresse), status="Kanal ungueltig")
        return

    # Programm in Schleife -> Auf Änderung prüfen -> bei Änderung senden + Neuen Status in Variable schreiben
    # Adresse 0x20-0x23 -> Index 0&1 / 2&3 / 4&5 / 6&7
    # Erster Value - Bank A, Zweiter = Bank B
    wertAltA=0
    wertAltB=0
    idx = (adresse - 0x20) * 2  # Berechne Index basierend auf Adresse
    wertAltA = stat_in[kanal][idx]
    wertAltB = stat_in[kanal][idx + 1]
    
    try:
        # GPIO A+B Lesen und String bauen:
        wertA=plexer.readByteData(kanal,adresse,gpioA)
        wertB=plexer.readByteData(kanal,adresse,gpioB)
        if wertAltA!=wertA or wertAltB!=wertB or manual==1:
            # Unterschied, Senden!
            iIn = [wertB, wertA]
            i=int.from_bytes(iIn,"big")
            publish_command_event("SAI", channel=kanal, address=hex(adresse), value=i, status="OK")

        # Erneut lesen, auf Änderung prüfen:
        wertA2=plexer.readByteData(kanal,adresse,gpioA)
        wertB2=plexer.readByteData(kanal,adresse,gpioB)
        if wertA2!=wertA or wertB2!=wertB:
            iIn = [wertB2, wertA2]
            i=int.from_bytes(iIn,"big")
            publish_command_event("SAI", channel=kanal, address=hex(adresse), value=i, status="OK")
            wertA=wertA2
            wertB=wertB2
        publish_input_bits(kanal, adresse, int.from_bytes([wertB, wertA], "big"))        
    except OSError as err:
        publish_command_event("SAI", channel=kanal, address=hex(adresse), status="IO Error SAI")
        log("I/O error: {0}".format(str(err)),"ERROR")
    except Exception as e:
        publish_command_event("SAI", channel=kanal, address=hex(adresse), status="Fehler SAI", message=str(e))
        log(f"Fehler Input lesen Kanal {kanal}, Addr {hex(adresse)}: {e}","ERROR")
    finally:
        # Status speichern
        stat_in[kanal][idx] = wertA
        stat_in[kanal][idx + 1] = wertB
        
        
        

def modulSuche(delete=0):
    """Sucht nach I2C-Modulen auf allen Kanälen"""
    # Daten löschen:
    if delete==1:
        for kanal in range(3):
            modules['in'][kanal].clear()
            modules['out'][kanal].clear()
            modules['pwm'][kanal].clear()
            modules['rgbw'][kanal].clear()
            modules['ana'][kanal].clear()
        input_state_cache.clear()
        output_state_cache.clear()
        pwm_state_cache.clear()
        analog_state_cache.clear()

    for kanalSearch in range(3):        
        log("Suche Bus: {0} Kanal: {1}".format(bus,kanalSearch))
        tmpIN=""
        tmpOut=""
        tmpRGBW=""
        tmpPWM=""
        tmpUnb=""
        tmpANA=""
        for device in range(128):
            try:
                if (plexer.readByte(kanalSearch,device,quiet=True)!= None):
                    if device!=mux and device!=DS2482.I2C_ADDR:
                        if device>=0x20 and device <=0x23:
                            log("GeCoS 16 In : Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpIN=tmpIN+hex(device)+";"
                            if device not in modules['in'][kanalSearch]:
                                if not set_input_konfig(kanalSearch,device):
                                    log("Kanal {0} Adresse {1}: Modul antwortet, laesst sich aber nicht konfigurieren - wird ignoriert".format(kanalSearch,hex(device)),"ERROR")
                                    continue
                                modules['in'][kanalSearch].append(device)
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="IN")
                        elif device>=0x24 and device <=0x27:
                            log("GeCoS 16 OUT: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpOut=tmpOut+hex(device)+";"
                            if device not in modules['out'][kanalSearch]:
                                if not set_output_konfig(kanalSearch,device):
                                    log("Kanal {0} Adresse {1}: Modul antwortet, laesst sich aber nicht konfigurieren - wird ignoriert".format(kanalSearch,hex(device)),"ERROR")
                                    continue
                                modules['out'][kanalSearch].append(device)
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="OUT")
                        elif device>=0x50 and device <=0x57:
                            log("GeCoS 16 PWM: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpPWM=tmpPWM+hex(device)+";"
                            if device not in modules['pwm'][kanalSearch]:
                                if not set_pwm_konfig(kanalSearch,device):
                                    log("Kanal {0} Adresse {1}: Modul antwortet, laesst sich aber nicht konfigurieren - wird ignoriert".format(kanalSearch,hex(device)),"ERROR")
                                    continue
                                modules['pwm'][kanalSearch].append(device)
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="PWM")
                        elif device>=0x58 and device <=0x5f:
                            log("GeCoS 16 RGBW: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpRGBW=tmpRGBW+hex(device)+";"
                            if device not in modules['rgbw'][kanalSearch]:
                                if not set_pwm_konfig(kanalSearch,device):
                                    log("Kanal {0} Adresse {1}: Modul antwortet, laesst sich aber nicht konfigurieren - wird ignoriert".format(kanalSearch,hex(device)),"ERROR")
                                    continue
                                modules['rgbw'][kanalSearch].append(device)
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="RGBW")
                        elif device>=0x68 and device <=0x6b:
                            log("GeCoS Analog4: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            tmpANA=tmpANA+hex(device)+";"
                            if device not in modules['ana'][kanalSearch]:
                                modules['ana'][kanalSearch].append(device)
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="ANA")
                        else:
                            tmpUnb=tmpUnb+hex(device)+";"
                            log("GeCoS Unbekanntes Gerät: Kanal: {0} Adresse: {1}".format(kanalSearch,hex(device)))
                            publish_command_event("MOD", channel=kanalSearch, address=hex(device), module_type="UNB")
            except Exception as e:
                log(f"Fehler beim Scannen von Device {device} auf Kanal {kanalSearch}: {e}", "DEBUG")
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16IN',tmpIN)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16OUT',tmpOut)
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'UNBEKANNT',tmpUnb)                
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16PWM',tmpPWM)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOSANA4',tmpANA)  
        configSchreiben('Module Bus {0}'.format(str(kanalSearch)),'GECOS16RGBW',tmpRGBW)
    publish_command_event("MOD", channel=0, address="0", status="END")
    publish_ha_discovery()
        
def bit_from_string(string, index):
    i=int(string)
    return i >> index & 1

if __name__ == '__main__':
    # Konfig Werte MCP:
    log("Script gestartet - Version {0} ({1})".format(__version__, os.path.abspath(__file__)), "ERROR")
    
    # ArgParser:
    parser = argparse.ArgumentParser(description='GeCoS-Server - Gebäudecontrol System')
    parser.add_argument('--debug', '-d', help='Aktiviert Debug-Ausgaben', action='store_true')
    parser.add_argument('--mqtt-host', default=os.getenv('MQTT_HOST', '127.0.0.1'), help='MQTT Broker Host (Default: 127.0.0.1)')
    parser.add_argument('--mqtt-port', type=int, default=int(os.getenv('MQTT_PORT', '1883')), help='MQTT Broker Port (Default: 1883)')
    parser.add_argument('--mqtt-username', default=os.getenv('MQTT_USERNAME'), help='MQTT Benutzername')
    parser.add_argument('--mqtt-password', default=os.getenv('MQTT_PASSWORD'), help='MQTT Passwort')
    parser.add_argument('--mqtt-base-topic', default=os.getenv('MQTT_BASE_TOPIC', 'gecos/server'), help='MQTT Basistopic, z.B. gecos/server')
    parser.add_argument('--mqtt-client-id', default=os.getenv('MQTT_CLIENT_ID'), help='MQTT Client ID (optional)')
    parser.add_argument('--mqtt-keepalive', type=int, default=int(os.getenv('MQTT_KEEPALIVE', '60')), help='MQTT Keepalive Sekunden (Default: 60)')
    parser.add_argument('--ha-discovery', action='store_true', help='MQTT Discovery für Home Assistant aktivieren')
    parser.add_argument('--ha-prefix', default=os.getenv('HA_PREFIX', 'homeassistant'), help='Home Assistant Discovery Prefix')
    parser.add_argument('--device-name', default=os.getenv('GECOS_DEVICE_NAME', 'GeCoS Server'), help='Anzeigename des Geräts für Home Assistant')
    parser.add_argument('--ow-interval', type=int, default=int(os.getenv('OW_INTERVAL', '30')), help='Abfrageintervall der OneWire-Geräte in Sekunden, 0 deaktiviert das Polling (Default: 30)')
    parser.add_argument('--ow-no-active-pullup', action='store_true', help='Aktiven Pull-up des DS2482 abschalten (nur bei starkem externen Pull-up sinnvoll)')
    args = parser.parse_args()
    if not args.ha_discovery:
        env_discovery = os.getenv('HA_DISCOVERY', '').lower()
        if env_discovery in ('1', 'true', 'yes'):
            args.ha_discovery = True
    
    if args.debug:
        print_debug = True
        logger.setLevel(logging.DEBUG)

    bus = 1  # 0 für rev1 boards etc.
    mux=0x71
    kanal=0
    bankAKonfig=0x00
    bankBKonfig=0x01
    outputKonfig=0x00
    inputKonfig=0xFF
    IOCONA=0x0A
    IOCONB=0x0B
    DEFVALA=0x06
    DEFVALB=0x07
    INTCONA=0x08
    INTCONB=0x09
    GPPUA=0x0C
    GPPUB=0x0D
    IPOLA=0x02
    IPOLB=0x03
    GPINTENA=0x04
    GPINTENB=0x05    
    intBankA=0x0E
    intBankB=0x0F
    intcapA=0x10
    intcapB=0x11
    gpioA=0x12
    gpioB=0x13
    bankA=0x14
    bankB=0x15
    aOutHex = [0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80]
    #Konfig:    
    #paketLaenge=1024
    freqStd=100
    
    #Interrupt Ports:
    intKanal0=17
    intKanal1=18
    intKanal2=27
    
    #Config lesen:
    configSchreiben('Allgemein','x','x')
   
    #MUX initialisieren:
    log("Bus:" + str(bus) + " Kanal:" + str(kanal))
    plexer = multiplex(bus)

    try:
        init_mqtt(args)
    except Exception:
        log("MQTT Initialisierung fehlgeschlagen, Programm wird beendet", "ERROR")
        sys.exit(1)

    log(datetime.now())
    
    #Modulsuche:
    modulSuche(1)
    
    #OneWire:
    dsOW = DS2482()
    #OneWire-Bus scannen, bei HA anmelden und zyklisches Auslesen starten:
    OWSearchDevice()
    threading.Thread(target=ow_poll_loop, daemon=True).start()
    #RTC Lesen (optional - ohne bestueckte RTC laeuft der Server normal weiter):
    ds = DS1307(plexer, 0x68)
    try:
        rtctime = ds.read_datetime()
        temp = ds.read_temp()
        log ("DS3231 Date: {0} Temp: {1} ".format(rtctime.strftime("%d.%m.%Y %H:%M:%S"),str(temp)))
    except Exception as exc:
        log(f"RTC auf Kanal 3 / 0x68 nicht lesbar, wird uebersprungen: {exc}", "WARNING")
    while True:
        try:
            # Alle Eingänge lesen
            time.sleep(0.01)
            # Schleife für Eingang Lesen:
            for kanal in range(3):
                for device in modules['in'][kanal]:
                    try:
                        read_input(kanal, device)
                    except Exception as e:
                        log(f"Fehler beim Lesen von Input Kanal {kanal}, Device {hex(device)}: {e}", "DEBUG")
        except KeyboardInterrupt:
            log("Server wird beendet...", "INFO")
            break

    if mqtt_client:
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except Exception:
            pass


