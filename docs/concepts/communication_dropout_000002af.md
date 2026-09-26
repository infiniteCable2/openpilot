# Kommunikationsaussetzer in `d4dd69160a48f11f/000002af--eac73cd1d8`

Ausgewertet wurden die qlogs aller 37 Segmente und die vollständigen rlogs der Segmente 0 sowie 18–20. Die folgenden `+`-Zeiten beziehen sich auf die gemeinsame Segmentbasis `414152447454` ns der Segmente 1–36; der erste Logeintrag in Segment 0 liegt 139,877 s davor. Die sichtbare Warnung bei +1174,136 s liegt damit ungefähr 21:54 nach dem ersten Logeintrag der Route.

## Persönlichkeitswechsel und nativer Params-Stillstand

Der erste der beobachteten Tastendrücke bei +1165,965 s setzte die Persönlichkeit von 1 auf 0. `Params.put` kehrte nach 0,26 ms zurück. Der asynchrone native Schreibvorgang für `LongitudinalPersonality` begann nahezu gleichzeitig und dauerte **11,381 s**, davon **11,287 s im `fsync` der temporären Datei**. Der parallel laufende Params-Lesethread setzte den angezeigten Wert nach 24 ms wieder von 0 auf 1 zurück; dasselbe geschah bei vier weiteren Tastendrücken bis +1172,028 s. Erst bei +1177,261 s, direkt nach Ende des `fsync`, las er den neuen Wert 0. Das belegt den bereits vermuteten Rücksprung durch Lesen vor Abschluss der asynchronen Persistierung.

Parallel blockierten verschiedene Prozesse in unterschiedlichen nativen Dateioperationen:

| Operation | Beginn ungefähr | Dauer | Langsame Phase |
| --- | ---: | ---: | --- |
| `LongitudinalPersonality` put | +1165,971 s | 11,381 s | `fsync` 11,287 s |
| `LagdValueCache` put | +1167,867 s | 9,566 s | `mkstemp` 9,366 s |
| `EnforceTorqueControl` remove | +1168,066 s | 9,187 s | Öffnen der Params-`.lock` 9,169 s |
| `Offroad_OSMUpdateRequired` remove | +1168,635 s | 8,599 s | Öffnen der `.lock` 8,599 s |
| `Offroad_TemperatureTooHigh` remove | +1169,605 s | 7,630 s | Öffnen der `.lock` 7,630 s |
| `LiveTorqueParameters` / `LiveCurvatureParameters` put | +1173,66 s | je etwa 3,7 s | `mkstemp` je etwa 3,5 s |

Das ist **kein bloßes Warten auf `flock`**: Mehrere Prozesse hingen vor der Sperre bereits in `open` oder `mkstemp`, während der erste Params-Schreiber in `fsync` hing. In Segment 19 fand der Inspector keinen Betriebssystemlog mit offensichtlichem Dateisystem-, UFS-/MMC- oder I/O-Fehler. Die Daten belegen erneut eine systemweite Verzögerung im Datei-/Speicherpfad, identifizieren aber weder den auslösenden Prozess noch einen Hardwaredefekt sicher.

## Sichtbare Warnung

`hardwared` veröffentlichte `deviceState` von +1169,585 bis +1177,588 s nicht: **8,003 s Lücke**. Der betroffene Zyklus dauerte 7,780 s bei nur 19,56 ms Thread-CPU; 7,631 s entfielen auf `set_offroad_alert_if_changed("Offroad_TemperatureTooHigh", false, ...)`, das in `Params().remove` die `.lock`-Datei öffnete. Die Warnung `commIssue/softDisable` begann bei +1174,136 s, als `deviceState` 5,011 s alt war, und verschwand bei +1178,142 s. `managerState` fiel zeitweise zusätzlich bei der Frequenzprüfung durch. `modelV2`, `longitudinalPlan`, `controlsState` und `pandaStates` liefen im Warnfenster weiter; `lanefulActive` war falsch. Der `Offroad_TiciSupport`-Aufruf aus der vorherigen Route benötigte hier nur 0,03 ms. Die vorige Reduktion dieses Aufrufs griff also, während weitere wiederholte Params-Entfernungen bestehen blieben.

In rlog und qlog fehlen `carState` und `carControl` jeweils für etwa 9,5 s ab +1167,7 s. Gleichzeitig laufen `carOutput`, `carStateSP`, `carStateIC` und `controlsState` weiter. Außerdem wurden weitere Persönlichkeitstastendrücke während dieser Loglücke verarbeitet; `selfdrived.carStateMissing` wurde nicht geloggt. Das spricht für einen selektiven Verlust bei der Aufzeichnung oder einer einzelnen Subscription, belegt aber nicht, an welchem Punkt diese beiden Nachrichten verloren gingen. Die gezählten SPI-NACKs treten auch vor und nach der Warnung auf; im Fokusfenster um +1174,1 s wurde keiner geloggt. Eine SPI-Ursache des Params-Stillstands ist damit nicht nachgewiesen.

## Konsequenz

Der neue Datenpunkt bestätigt den Persönlichkeits-Rücksprung und den Pfad `langsame Params-Dateioperationen → blockiertes hardwared → fehlendes deviceState → commIssue`. Drei vermeidbare Wiederholungen wurden identifiziert: wechselnder Temperaturtext löste trotz inaktiver Warnung erneutes `remove` aus; `mapd` entfernte eine unveränderte OSM-Warnung jede Sekunde; die Sunny-UI entfernte bei jeder 5-Hz-Parameteraktualisierung nicht vorhandene, für das Fahrzeug unzulässige Schlüssel. Außerdem schrieb `lagd` `LagdValueCache` alle drei Sekunden auch bei unverändertem Wert. Die folgende Revision unterbindet die wiederholten Entfernungen und unveränderten Cache-Schreibvorgänge. Sie löst die Ursache der 9–11-s-Dateioperationen noch nicht.

Für den separaten Anfangsaussetzer ist die neue Beobachtung des Fahrers wichtig: Er tritt beim ersten klaren Erkennen beider Spurgrenzen mit aktiviertem Laneful auf. Das passt zu der früher gemessenen einmaligen Verzögerung in `lane_target()` auf Route `000002ae` (+82,961 s, 96,24 ms einschließlich eines schweren Seitenfehlers). `lane_target()` erreicht erst bei hinreichend plausiblen Spurgrenzen die NumPy-Geometrie- und `polyfit`-Berechnung; der erste solche Aufruf könnte deshalb einmalig Seiten laden oder Bibliotheken initialisieren, noch bevor `lanefulActive` nach der Anlaufzeit wahr wird. Das ist eine Hypothese, kein Nachweis. In dieser Route `000002af` war Laneful in den qlog-Stichproben nie aktiv und in Segment 19 wurde kein `lane_target()`-Aufruf gemessen; der Persönlichkeitsaussetzer ist davon unabhängig. Der erste vollständige `lane_target()`-Aufruf und seine `polyfit`-Phase sollten bei einer erneuten Fahrt mit Laneful ab dem Startpunkt separat geprüft werden.

Nachvollziehbar mit `PYTHONPATH=. python tools/scripts/inspect_comm_issue_route.py d4dd69160a48f11f/000002af--eac73cd1d8 18 19 20` sowie `--focus 1174.1` auf Segment 19. `--qlog-events` erlaubt eine schnelle Ereignissuche über alle Segmente, ersetzt für Nachrichtenlücken jedoch keine rlogs.
