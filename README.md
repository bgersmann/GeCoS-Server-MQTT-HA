# GeCoS-Server (MQTT Edition)

GeCoS-Server steuert die GeCoS-I2C- und OneWire-Module eines Gebaeudebus-Systems und stellt alle Zustaende sowie Befehle komplett ueber MQTT bereit. Optional erzeugt der Dienst automatisch Home-Assistant-Discovery-Payloads, damit Eingaenge, Ausgaenge, PWM-Dimmer, Analogsensoren und OneWire-Geraete ohne manuelle YAML-Konfiguration erscheinen.

## Highlights
- Native MQTT-Integration mit getrennten Topics fuer Status, Kommandos und Sensordaten
- Home Assistant MQTT Discovery fuer Binary-Sensoren, Switches, Light-Dimmer, Analogsensoren und OneWire-Temperatursensoren
- Unterstuetzung fuer MCP23017 (16 IN/OUT), PCA9685 (PWM & RGBW), MCP3424 (Analog), DS2482 + OneWire-Geraete sowie DS1307/DS3231 RTC
- Zyklisches Auslesen des OneWire-Bus mit eigener Verfuegbarkeitsmeldung pro Sensor
- Prozentbasierte PWM-Steuerung inklusive automatischer Umrechnung zwischen 0-100 Prozent und 12-Bit-Werten
- Rueckwaertskompatible "raw"-Kommandos fuer bestehende Integrationen, jetzt ueber MQTT

## Anforderungen
- Raspberry Pi (oder kompatibles Linux-System) mit aktivem I2C-Bus
- Python 3.8 oder neuer
- Pakete: `smbus` (bzw. `python3-smbus`) und `paho-mqtt`
- MQTT-Broker (z. B. Mosquitto auf Home Assistant)

Installiere die Python-Abhaengigkeiten typischerweise mit:

```bash
sudo apt install python3-smbus python3-pip
pip3 install --upgrade paho-mqtt
```

## Schnellstart
1. Repository klonen und Skript kopieren:
   ```bash
   git clone https://github.com/bgersmann/GeCoS-Server-MQTT.git
   cd GeCoS-Server-MQTT
   sudo cp GeCoS-Server.py /usr/local/bin/
   sudo chmod +x /usr/local/bin/GeCoS-Server.py
   ```
2. Server manuell testen:
   ```bash
   python3 /usr/local/bin/GeCoS-Server.py \
     --mqtt-host homeassistant.local \
     --mqtt-base-topic home/gecos \
     --ha-discovery
   ```
3. Sobald die MQTT-Verbindung steht, erscheinen die Module automatisch unter `homeassistant/...` (falls Discovery aktiv ist).

## Konfiguration
### Kommandozeilenoptionen
| Option | Beschreibung | Environment Override |
|--------|--------------|----------------------|
| `--debug` / `-d` | Aktiviert detaillierte Logausgaben | – |
| `--mqtt-host` | Hostname oder IP des MQTT-Brokers (Default `127.0.0.1`) | `MQTT_HOST` |
| `--mqtt-port` | Broker-Port (Default `1883`) | `MQTT_PORT` |
| `--mqtt-username` / `--mqtt-password` | Zugangsdaten fuer den Broker | `MQTT_USERNAME`, `MQTT_PASSWORD` |
| `--mqtt-base-topic` | Einstiegstopic, z. B. `gecos/server` | `MQTT_BASE_TOPIC` |
| `--mqtt-client-id` | MQTT Client-ID (ansonsten hostname-basiert) | `MQTT_CLIENT_ID` |
| `--mqtt-keepalive` | Keepalive in Sekunden (Default `60`) | `MQTT_KEEPALIVE` |
| `--ha-discovery` | Schaltet Home Assistant Discovery frei | `HA_DISCOVERY` |
| `--ha-prefix` | Discovery-Prefix (Default `homeassistant`) | `HA_PREFIX` |
| `--device-name` | Anzeigename innerhalb von Home Assistant | `GECOS_DEVICE_NAME` |
| `--ow-interval` | Abfrageintervall der OneWire-Geraete in Sekunden, `0` deaktiviert das Polling (Default `30`) | `OW_INTERVAL` |

### Systemd-Service (Beispiel)

```ini
[Unit]
Description=GeCoS MQTT Server
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /usr/local/bin/GeCoS-Server.py \
  --mqtt-host homeassistant.local \
  --mqtt-base-topic home/gecos \
  --ha-discovery
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Aktivierung wie gewohnt via `sudo systemctl enable --now gecos.service`.

## MQTT-Topics
Alle Topics haengen unter dem konfigurierten Basistopic (Standard `gecos/server`). Die wichtigsten Pfade:

| Topic | Inhalt |
|-------|--------|
| `<base>/status` | Verfuegbarkeit (`online`/`offline`, retained) |
| `<base>/state/<event>` | Strukturierte Events (z. B. `MOD`, `SOM`, Fehler) als JSON |
| `<base>/inputs/<kanal>/<adresse>/<bit>` | Binary-Sensor-State (`ON`/`OFF`) pro Eingangspin |
| `<base>/outputs/<kanal>/<adresse>/<bit>` | Switch-Status der Ausgaenge |
| `<base>/pwm/<kanal>/<adresse>/<channel>` | JSON `{state, brightness}` fuer PWM-Kanaele (0-100 %) |
| `<base>/analog/<kanal>/<adresse>/<channel>` | Analoge Messwerte als String |
| `<base>/onewire/<adresse>/temperature` | Temperatur in °C (DS18B20, DS18S20, MAX31850) |
| `<base>/onewire/<adresse>/a` bzw. `/b` | Schaltzustand der beiden DS2413-Ausgaenge (`ON`/`OFF`) |
| `<base>/onewire/<adresse>/status` | Verfuegbarkeit des einzelnen OneWire-Geraets (`online`/`offline`) |
| `<base>/command/...` | Befehle an den Server (siehe unten) |

Die OneWire-Adresse hat die Form `<family>-<crc+serie>`, z. B. `28-a601183074cbff`.

### Befehls-Topics
- `command/output/<kanal>/<adresse>/<bit>` → Payload `ON`/`OFF` (oder `1`/`0`) schaltet einen Ausgang.
- `command/pwm/<kanal>/<adresse>/<channel>` → Payload Prozentwert, 0‑4095 oder JSON `{ "state": "ON", "brightness": 42 }` dimmt einen PWM-Kanal.
- `command/onewire/<adresse>/<a|b>` → Payload `ON`/`OFF` schaltet einen einzelnen DS2413-Ausgang; der jeweils andere Pin bleibt unveraendert.
- `command/raw` → Fruehere geschweifte Kommando-Strings, jetzt als MQTT-Nachricht (`{SAO;0;0x24}` usw.).

## Home Assistant Integration
Aktiviere `--ha-discovery`, damit der Server automatisch folgende Entitaeten registriert:
- Binary Sensoren fuer alle Eingaenge
- Switches fuer alle Ausgaenge
- Light-Dimmer (Brightness-Scale 0‑100 %) fuer PWM-Module
- Sensoren fuer Analogeingaenge
- Temperatursensoren fuer DS18B20, DS18S20 und MAX31850 am OneWire-Bus
- Je zwei Switches pro DS2413 (PIO A und PIO B)

Alle Entities teilen sich die Availability `<base>/status`. Unique IDs basieren auf Kanal, Adresse und Port, wodurch spaetere Re-Pairings ohne Duplikate funktionieren.

### OneWire
Der OneWire-Bus wird beim Start einmal durchsucht (`OWS`), die gefundenen Geraete landen in `Config.cfg` und werden bei Home Assistant angemeldet. Anschliessend liest ein Hintergrund-Thread alle Geraete im Abstand von `--ow-interval` Sekunden aus und veroeffentlicht die Werte retained. Ein erneutes `OWS`-Kommando aktualisiert die Geraeteliste und die Discovery-Eintraege zur Laufzeit.

Zusaetzlich zur globalen Availability besitzt jedes OneWire-Geraet ein eigenes `status`-Topic. Schlaegt ein Lesevorgang fehl (Rueckgabewert `-85`), wird das Geraet auf `offline` gesetzt und in Home Assistant als *nicht verfuegbar* angezeigt, statt einen falschen Messwert zu liefern.

DS2413-Ausgaenge sind Open-Drain: Das Latch-Bit `0` bedeutet "Ausgang aktiv". Der Server bildet das nach aussen als `ON` ab, ein `ON` in Home Assistant schaltet den Transistor also durch.

## Unterstuetzte Module & Geraete
- MCP23017: 16-fach Ein- oder Ausgangsmodule pro I2C-Adresse
- PCA9685: PWM- und RGBW-Treiber inklusive Konfiguration
- MCP3424: 4-fach Analog-Digital-Wandler mit konfigurierbarer Aufloesung
- DS2482 + DS18B20/DS18S20/DS2413/MAX31850: OneWire-Sensorik
- DS1307/DS3231: RTC-Auslesung fuer Zeit- und Temperaturwerte

## Fehlersuche
- `--debug` aktiviert ausfuehrliches Logging (inklusive I2C-Fehlern).
- MQTT-Konnektivitaet testen: `mosquitto_sub -h <broker> -t "gecos/#"`.
- Remote-I/O-Fehler beim I2C-Scan sind meist harmlos; bei Bedarf Log-Level reduzieren.

Weitere Optimierungsideen findest du in [OPTIMIERUNGEN.md](OPTIMIERUNGEN.md).
